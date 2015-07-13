#include <kinects_human_tracking/kinect_human_tracking.hpp>

/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then track him using a Kalman filter
**/

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_human_tracking");
  ros::NodeHandle nh, nh_priv("~");
  tf_listener_ = new tf::TransformListener();
  sleep(0.5); //to make sure tf_listener is ready
  
  ROS_INFO("Initializing ...");
   
  // Get params topics and frames names
  string kinect_topic_name, robot_topic_name, clusters_topic_name, out_topic_name;
  nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  nh_priv.getParam("robot_topic_name",robot_topic_name);
  nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  nh_priv.getParam("out_topic_name",out_topic_name);
  nh_priv.getParam("voxel_size",voxel_size_);
  min_cluster_size_ = 200; //TODO Make this a param
  
  // Initialize PointClouds
  pcl_pc1_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  robot_pc_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
  clustered_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  pcl_pc1_->reserve(10000);
  robot_pc_->reserve(10000);
  clustered_cloud_->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_clustered_pub_ = nh.advertise<PointCloudSM>(clusters_topic_name, 1);
  human_pc_pub_ = nh.advertise<PointCloudSM>(out_topic_name, 1);
  cloud_mini_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("kinect_both/min_human_pt",1);
  dist_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("kinect_both/min_robot_pt",1);
  human_pose_pub_ = nh.advertise<geometry_msgs::PointStamped>("kinect_both/human_pose",1);
  human_pose_obs_pub_ = nh.advertise<geometry_msgs::PointStamped>("kinect_both/human_obs",1);
  
  message_filters::Subscriber<PCMsg> kinect_pc_sub(nh, kinect_topic_name, 1);
  message_filters::Subscriber<PCMsg> robot_pc_sub(nh, robot_topic_name, 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), kinect_pc_sub, robot_pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  // TODO Make these be params
  // Initialize Kalman filter
  Eigen::Vector2f jerk_std;
  jerk_std(0) = 0.01;
  jerk_std(1) = 0.01;
  Eigen::Vector2f measur_std;
  measur_std(0) = 0.01;
  measur_std(1) = 0.01;
  float delta_t = 1/30; //Assuming we should get about 30FPS
  Eigen::Matrix<float, 6, 1> x_k1;
  x_k1.fill(0.0);
  Eigen::Matrix<float, 6, 6> init_cov;
  init_cov.fill(0.0);
  kalman_.init(jerk_std, measur_std, delta_t, x_k1, init_cov);
  
  last_human_pos_(0) = 0.0;
  last_human_pos_(1) = 0.0;
  
  ROS_INFO("Human tracker ready !");
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*human_pc_msg, *pcl_pc1_);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*pcl_pc1_, *pcl_pc1_, indices);
  
  // TODO Load clipping rules from XML or launch file
  // Clip pointcloud, here just to remove everything below z==0 
  vector<ClippingRule> clipping_rules;
  ClippingRule ground_remove;
  ground_remove.axis = "z";
  ground_remove.op = "GT";
  ground_remove.val = 0.0;
  clipping_rules.push_back(ground_remove);
  pc_clipping(pcl_pc1_, clipping_rules , pcl_pc1_);
  
  // Downsampling of the kinects cloud  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_downsampling(pcl_pc1_, pc_filtered, voxel_size_);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices = pc_clustering(pc_filtered, 2*voxel_size_ ,pc_filtered);
  pc_clustered_pub_.publish(*pc_filtered);
  
  // TODO Before gettting stats, the poincloud need to be in the 'base_link' frame
  // Getting heights for all clusters
  std::vector<double> cluster_heights; 
  vector<ClusterStats> stats = get_clusters_stats (pc_filtered , cluster_indices);
  for(int i=0; i<stats.size();i++){
    std::string tmp = boost::lexical_cast<std::string>(stats[i].max-stats[i].min);
    double cluster_height = (double)atof(tmp.c_str());
    cluster_heights.push_back(cluster_height);
  }
  
  // TODO  Make height be a param
  // Selecting only the clusters with height superior to 0.3
  int j = -1, nb_clusters_left = 0;
  pcl::copyPointCloud(*pc_filtered, *clustered_cloud_);
  clustered_cloud_->points.clear();
  clustered_cloud_->width = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    j++;
    if(cluster_heights[j]<0.3)
      continue;
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      clustered_cloud_->points.push_back(pc_filtered->points[*pit]); 
      clustered_cloud_->width++;
    }
    nb_clusters_left++;
  }

  // Transform the robot's pointCloud
  sensor_msgs::PointCloud2 pcl_out; 
  tf_listener_->waitForTransform(clustered_cloud_->header.frame_id, robot_pc_msg->header.frame_id, robot_pc_msg->header.stamp, ros::Duration(5.0)); 
  pcl_ros::transformPointCloud(clustered_cloud_->header.frame_id, *robot_pc_msg, pcl_out, *tf_listener_); 
  
  pcl::fromROSMsg(pcl_out, *robot_pc_);
  
  // Downsample the robot's cloud
  pc_downsampling(robot_pc_,robot_pc_,voxel_size_);
  
  // Get closest cluster to the robot
  double min_dist;
  geometry_msgs::PointStamped pt_human, pt_robot;
  get_closest_cluster_to_robot(clustered_cloud_,cluster_indices, robot_pc_, clustered_cloud_, min_dist, pt_human, pt_robot);
  
  // min_dist!=0 means we have at least one cluster
  if (nb_clusters_left>0){
    // Publish human pointCloud
    clustered_cloud_->width = clustered_cloud_->points.size();
    human_pc_pub_.publish(*clustered_cloud_);
    
    // Publish minimum points 
    cloud_mini_pt_pub_.publish<geometry_msgs::PointStamped>(pt_human);
    dist_pt_pub_.publish<geometry_msgs::PointStamped>(pt_robot);
    
    // TODO Save the transform once and for all so we don't have to load it every time
    // Transform the cloud to the world frame before computing the stats
    tf::StampedTransform stampedTransform;
    tf_listener_->waitForTransform(robot_pc_msg->header.frame_id, clustered_cloud_->header.frame_id, ros::Time::now(), ros::Duration(5.0)); 
    tf_listener_->lookupTransform(robot_pc_msg->header.frame_id, clustered_cloud_->header.frame_id, ros::Time(0.0), stampedTransform);
    tf::Transform transform;
    transform.setBasis(stampedTransform.getBasis());
    transform.setOrigin(stampedTransform.getOrigin());
    transform.setRotation(stampedTransform.getRotation());
    pcl_ros::transformPointCloud(*clustered_cloud_, *clustered_cloud_, transform); 
    
    // Get stats on human's pointCloud
    ClusterStats human_stats = get_cluster_stats(clustered_cloud_);
    
    // Get pose observation from the stats
    Eigen::Vector2f obs;
    obs(0) = human_stats.median(0);
    obs(1) = human_stats.median(1);
    Eigen::Matrix<float, 6, 1>  x_k1;
    x_k1.fill(0.);
    x_k1(0,0) = obs(0);
    x_k1(1,0) = obs(1);
    
    // Publish human observation
    geometry_msgs::PointStamped human_obs;
    human_obs.header.frame_id="base_link"; //TODO no hard coding here !!!
    human_obs.point.x = obs(0);
    human_obs.point.y = obs(1);
    human_pose_obs_pub_.publish<geometry_msgs::PointStamped>(human_obs);
    
    // TODO Make the distance a parameter
    // If the new observation is too far from the previous one, reinitialize
    if ( (last_human_pos_-obs).norm() > 0.2){
      kalman_.init(Eigen::Vector2f(2.5,2.5), Eigen::Vector2f(0.01,0.01), -1, x_k1);
      ROS_INFO("New human pose to far. Reinitializing !");
    }
  
    // Feed the Kalman filter with the observation and get back the estimated state
    float delta_t = 0.03;
    Eigen::Matrix<float, 6, 1> est;
    kalman_.estimate(obs, delta_t, est); 
    
    // Save new estimated pose
    last_human_pos_(0) = est(0);
    last_human_pos_(1) = est(1);
    
    // Publish human estimated pose
    geometry_msgs::PointStamped human_pose;
    human_pose.header.frame_id="base_link";
    human_pose.point.x = last_human_pos_(0);
    human_pose.point.y = last_human_pos_(1);
    human_pose_pub_.publish<geometry_msgs::PointStamped>(human_pose);
  }
}