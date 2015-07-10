#include <kinects_human_tracking/kinect_human_tracking.hpp>

/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then track him using a Kalman filter
**/

void get_closest_cluster_to_robot(PointCloudSM::Ptr human_clustered_pc, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr robot_pc, PointCloudSM::Ptr pc_out, double& min_dist, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min);

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_human_tracking");
  ros::NodeHandle nh, nh_priv("~");
  tf_listener = new tf::TransformListener();
  sleep(0.5); //to make sure tf_listener is ready
   
  // Get params topics and frames names
  string kinect_topic_name, robot_topic_name, clusters_topic_name, out_topic_name;
  nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  nh_priv.getParam("robot_topic_name",robot_topic_name);
  nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  nh_priv.getParam("out_topic_name",out_topic_name);
  nh_priv.getParam("voxel_size",voxel_size_);
  min_cluster_size_ = 200;
  
  // Initialize PointClouds
  pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  robot_pc = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
  clustered_cloud = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  pcl_pc1->reserve(10000);
  robot_pc->reserve(10000);
  clustered_cloud->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_clustered_pub = nh.advertise<PointCloudSM>(clusters_topic_name, 1);
  human_pc_pub = nh.advertise<PointCloudSM>(out_topic_name, 1);
  cloud_mini_pt_pub = nh.advertise<geometry_msgs::PointStamped>("kinect_both/min_human_pt",1);
  dist_pt_pub = nh.advertise<geometry_msgs::PointStamped>("kinect_both/min_robot_pt",1);
  human_pose_pub = nh.advertise<geometry_msgs::PointStamped>("kinect_both/human_pose",1);
  human_pose_obs_pub = nh.advertise<geometry_msgs::PointStamped>("kinect_both/human_obs",1);
  
  message_filters::Subscriber<PCMsg> kinect_pc_sub(nh, kinect_topic_name, 1);
  message_filters::Subscriber<PCMsg> robot_pc_sub(nh, robot_topic_name, 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), kinect_pc_sub, robot_pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  // Initialize Kalman filter
  Eigen::Vector2f jerk_std;
  jerk_std(0) = 0.01;
  jerk_std(1) = 0.01;
  Eigen::Vector2f measur_std;
  measur_std(0) = 0.01;
  measur_std(1) = 0.01;
  float delta_t = 1/30;
  Eigen::Matrix<float, 6, 1> x_k1;
  x_k1.fill(0.0);
  Eigen::Matrix<float, 6, 6> init_cov;
  init_cov.fill(0.0);
  kalman_.init(jerk_std, measur_std, delta_t, x_k1, init_cov);
  
  last_human_pos_(0) = 0.0;
  last_human_pos_(1) = 0.0;
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*human_pc_msg, *pcl_pc1);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*pcl_pc1, *pcl_pc1, indices);
  
  // Clip pointcloud, here just to remove everything below z==0 
  vector<ClippingRule> clipping_rules;
  ClippingRule ground_remove;
  ground_remove.axis = "z";
  ground_remove.op = "GT";
  ground_remove.val = 0.0;
  clipping_rules.push_back(ground_remove);
  pc_clipping(pcl_pc1, clipping_rules , pcl_pc1);
  
  // Downsampling of the kinects cloud  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_downsampling(pcl_pc1, pc_filtered, voxel_size_);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices = pc_clustering(pc_filtered, 2*voxel_size_ ,pc_filtered);
  pc_clustered_pub.publish(*pc_filtered);
  
  // Getting heights for all clusters
  std::vector<double> cluster_heights; 
  vector<ClusterStats> stats = get_cluster_stats (pc_filtered , cluster_indices);
  for(int i=0; i<stats.size();i++){
    std::string tmp = boost::lexical_cast<std::string>(stats[i].max-stats[i].min);
    double cluster_height = (double)atof(tmp.c_str());
    cluster_heights.push_back(cluster_height);
  }
  
  // Selecting only the clusters with height superior to 0.3
  int j = -1, nb_clusters_left = 0;
  pcl::copyPointCloud(*pc_filtered, *clustered_cloud);
  clustered_cloud->points.clear();
  clustered_cloud->width = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    j++;
    if(cluster_heights[j]<0.3)
      continue;
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      clustered_cloud->points.push_back(pc_filtered->points[*pit]); 
      clustered_cloud->width++;
    }
    nb_clusters_left++;
  }

  // Transform the robot's pointCloud
  sensor_msgs::PointCloud2 pcl_out; 
  tf_listener->waitForTransform(clustered_cloud->header.frame_id, robot_pc_msg->header.frame_id, robot_pc_msg->header.stamp, ros::Duration(5.0)); 
  pcl_ros::transformPointCloud(clustered_cloud->header.frame_id, *robot_pc_msg, pcl_out, *tf_listener); 
  
  pcl::fromROSMsg(pcl_out, *robot_pc);
  
  // Downsample the robot's cloud
  pc_downsampling(robot_pc,robot_pc,voxel_size_);
  
  // Get closest cluster to the robot
  double min_dist;
  geometry_msgs::PointStamped pt_human, pt_robot;
  get_closest_cluster_to_robot(clustered_cloud, cluster_indices, robot_pc, clustered_cloud, min_dist, pt_human, pt_robot);
  
  // min_dist!=0 means we have at least one cluster
  if (nb_clusters_left>0){
    // Publish human pointCloud
    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;
    human_pc_pub.publish(*clustered_cloud);
    
    // Publish minimum points 
    cloud_mini_pt_pub.publish<geometry_msgs::PointStamped>(pt_human);
    dist_pt_pub.publish<geometry_msgs::PointStamped>(pt_robot);
    
    // TODO
    // Transform the cloud to the world frame before computing the stats
    tf::StampedTransform stampedTransform;
    tf_listener->waitForTransform(robot_pc_msg->header.frame_id, clustered_cloud->header.frame_id, ros::Time::now(), ros::Duration(5.0)); 
    tf_listener->lookupTransform(robot_pc_msg->header.frame_id, clustered_cloud->header.frame_id, ros::Time(0.0), stampedTransform);
    tf::Transform transform;
    transform.setBasis(stampedTransform.getBasis());
    transform.setOrigin(stampedTransform.getOrigin());
    transform.setRotation(stampedTransform.getRotation());
    pcl_ros::transformPointCloud(*clustered_cloud, *clustered_cloud, transform); 
    
    // Get stats on human's pointCloud
    ClusterStats human_stats = get_cluster_stats(clustered_cloud);
    
    Eigen::Vector2f obs;
    obs(0) = human_stats.median(0);
    obs(1) = human_stats.median(1);
    Eigen::Matrix<float, 6, 1>  x_k1;
    x_k1.fill(0.);
    x_k1(0,0) = obs(0);
    x_k1(1,0) = obs(1);
    
    geometry_msgs::PointStamped human_obs;
    human_obs.header.frame_id="base_link";
    human_obs.point.x = obs(0);
    human_obs.point.y = obs(1);
    human_pose_obs_pub.publish<geometry_msgs::PointStamped>(human_obs);
    
    cout << "Obs :"<<endl;
    cout << obs(0)<<endl;
    cout << obs(1)<<endl;
    cout << "Last pose :"<<endl;
    cout << last_human_pos_(0)<<endl;
    cout << last_human_pos_(1)<<endl;
    cout << "Norm diff : "<< (last_human_pos_-obs).norm()<<endl<<endl;
    
    if ( (last_human_pos_-obs).norm() > 0.2){
      kalman_.init(Eigen::Vector2f(2.5,2.5), Eigen::Vector2f(0.01,0.01), -1, x_k1);
      cout << "New human to far. Reinitializing "<<endl<<endl;
    }
  
    // Feed the Kalman filter with the observation and get back the estimated state
    float delta_t = 0.03;
    Eigen::Matrix<float, 6, 1> est;
    kalman_.estimate(obs, delta_t, est); 
    
    last_human_pos_(0) = est(0);
    last_human_pos_(1) = est(1);
    
    geometry_msgs::PointStamped human_pose;
    human_pose.header.frame_id="base_link";
    human_pose.point.x = last_human_pos_(0);
    human_pose.point.y = last_human_pos_(1);
    human_pose_pub.publish<geometry_msgs::PointStamped>(human_pose);
    
  }
}

void get_closest_cluster_to_robot(PointCloudSM::Ptr human_clustered_pc, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr robot_pc, PointCloudSM::Ptr pc_out, double& mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min){
  
  // Check there is at least 1 cluster and that the robot cloud is published
  if ((cluster_indices.size() < 1) || (robot_pc->points.size() < 1))
    return;
  
  double min_dist;
  vector<double> min_dists;
  for(int i=0; i<cluster_indices.size();i++){
    pc_to_pc_min_dist(human_clustered_pc, robot_pc, min_dist, pc1_pt_min, pc2_pt_min);
    min_dists.push_back(min_dist);
  }
  
  mini = min_dists[0];
  int min_id = 0;
  for(int i=1; i<min_dists.size();i++){
    if(min_dists[i] < mini){
      mini = min_dists[i];
      min_id = i;
    } 
  }
  
  pcl::copyPointCloud(*human_clustered_pc,*pc_out);
  pc_out->points.clear();
  pcl::PointIndices idx = cluster_indices[min_id];
  for (std::vector<int>::const_iterator pit = idx.indices.begin(); pit != idx.indices.end(); ++pit){
      pc_out->points.push_back(human_clustered_pc->points[*pit]); 
      pc_out->width++;
  }
  
}