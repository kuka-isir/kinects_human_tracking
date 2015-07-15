#include <kinects_human_tracking/kinect_human_tracking.hpp>
/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then track him using a Kalman filter
 */

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_human_tracking");
  ros::NodeHandle nh, nh_priv("~");
  tf_listener_ = new tf::TransformListener();
  sleep(0.5); //to make sure tf_listener is ready
  
  ROS_INFO("Initializing ...");  
   
  // Get params topics and frames names
  string kinect_topic_name, robot_topic_name, clusters_topic_name, out_topic_name;
  XmlRpc::XmlRpcValue clipping_rules_bounds;
  nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  nh_priv.getParam("robot_topic_name",robot_topic_name);
  nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  nh_priv.getParam("out_topic_name",out_topic_name);
  nh_priv.getParam("voxel_size",voxel_size_);
  nh_priv.getParam("min_cluster_size",min_cluster_size_);
  nh_priv.getParam("kinect_noise",kinect_noise_);
  nh_priv.getParam("process_noise",process_noise_);
  nh_priv.getParam("minimum_height",minimum_height_);
  nh_priv.getParam("max_tracking_jump",max_tracking_jump_);
  nh_priv.getParam("clipping_rules",clipping_rules_bounds);
  
  if (clipping_rules_bounds.size()>0){
    if (clipping_rules_bounds.size()%3)
      ROS_ERROR("Problem in defining the clipping rules.\n Use the following format:\n [x, GT, 1.0, y, LT, 3.1, ...]");
    else{
      clipping_rules_.resize(clipping_rules_bounds.size()/3);
      ClippingRule new_rule;
      for(int i=0; i<clipping_rules_bounds.size()/3;i++){
	new_rule.axis = static_cast<string>(clipping_rules_bounds[i*3]);//   clipping_rules_bounds[i*3];
	new_rule.op = static_cast<string>(clipping_rules_bounds[i*3+1]);//   clipping_rules_bounds[i*3];
	new_rule.val = static_cast<double>(clipping_rules_bounds[i*3+2]);//   clipping_rules_bounds[i*3];
	clipping_rules_.at(i) = new_rule;
      }
    }
    cout<<clipping_rules_.size()<< " clipping rules loaded"<<endl;
  }
  
  // Initialize PointClouds
  kinects_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  robot_pc_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
  clustered_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  kinects_pc_->reserve(10000);
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
  
  // Initialize Kalman filter
  Eigen::Matrix<float, 6, 1> x_k1;
  x_k1.fill(0.0);
  Eigen::Matrix<float, 6, 6> init_cov;
  init_cov.fill(0.0);
  kalman_.init(Eigen::Vector2f(kinect_noise_, process_noise_), Eigen::Vector2f(process_noise_ ,process_noise_), -1, x_k1, init_cov);
  
  // Initializing the human's position at the origin
  last_human_pos_ = Eigen::Vector2f(0.0, 0.0);
  
  ROS_INFO("Human tracker ready !");
  
  last_observ_time_ = ros::Time(0.0);
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*human_pc_msg, *kinects_pc_);
  pcl::fromROSMsg(*robot_pc_msg, *robot_pc_);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*kinects_pc_, *kinects_pc_, indices);
  
  // Clip pointcloud using the rules defined in params
  pc_clipping(kinects_pc_, clipping_rules_ , kinects_pc_);
  
  // Downsampling the two pointClouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_downsampling(kinects_pc_, pc_filtered, voxel_size_);
  pc_downsampling(robot_pc_, robot_pc_, voxel_size_);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices = pc_clustering(pc_filtered, 2*voxel_size_ ,pc_filtered);
  pc_clustered_pub_.publish(*pc_filtered);
  
  int nb_clusters = cluster_indices.size();
  
  if(minimum_height_ >0){
    // Getting heights for all clusters
    std::vector<double> cluster_heights; 
    vector<ClusterStats> stats = get_clusters_stats (pc_filtered , cluster_indices);
    for(int i=0; i<nb_clusters;i++){
      std::string tmp = boost::lexical_cast<std::string>(stats[i].max-stats[i].min);
      double cluster_height = (double)atof(tmp.c_str());
      cluster_heights.push_back(cluster_height);
    }

    // Selecting only the clusters with the minimum_height
    int j = -1;
    pcl::copyPointCloud(*pc_filtered, *clustered_cloud_);
    clustered_cloud_->points.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
      j++;
      if(cluster_heights[j]<minimum_height_){
	nb_clusters--;
	continue;
      }
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	clustered_cloud_->points.push_back(pc_filtered->points[*pit]); 
    }
    clustered_cloud_->width = clustered_cloud_->points.size();
  }
  else
    pcl::copyPointCloud(*pc_filtered, *clustered_cloud_); 
  
  // Get closest cluster to the robot
  get_closest_cluster_to_robot(clustered_cloud_, cluster_indices, robot_pc_, clustered_cloud_, last_min_dist_, last_human_pt_, last_robot_pt_);
  
  if (nb_clusters>0){
    // Publish human pointCloud
    human_pc_pub_.publish(*clustered_cloud_);
    
    // Publish minimum points 
    cloud_mini_pt_pub_.publish<geometry_msgs::PointStamped>(last_human_pt_);
    dist_pt_pub_.publish<geometry_msgs::PointStamped>(last_robot_pt_);
    
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
    human_obs.header.frame_id = robot_pc_msg->header.frame_id; 
    human_obs.point.x = obs(0);
    human_obs.point.y = obs(1);
    human_pose_obs_pub_.publish(human_obs);
    
    // If the new observation is too far from the previous one, reinitialize
    if ( (last_human_pos_-obs).norm() > max_tracking_jump_){
      kalman_.init(Eigen::Vector2f(kinect_noise_, process_noise_), Eigen::Vector2f(process_noise_ ,process_noise_), -1, x_k1);
      ROS_INFO("New human pose to far. Reinitializing !");
    }
  
    // Feed the Kalman filter with the observation and get back the estimated state
    float delta_t;
    if (last_observ_time_.sec == 0)
      delta_t = -1;
    else
      delta_t = ros::Time::now().sec - last_observ_time_.sec + (ros::Time::now().nsec - last_observ_time_.nsec)*pow(0.1,9);
    Eigen::Matrix<float, 6, 1> est;
    kalman_.estimate(obs, delta_t, est); 
    last_observ_time_ = ros::Time::now();
    
    // Save new estimated pose
    last_human_pos_(0) = est(0);
    last_human_pos_(1) = est(1);
    
    // Publish human estimated pose
    geometry_msgs::PointStamped human_pose;
    human_pose.header.frame_id = robot_pc_msg->header.frame_id;
    human_pose.point.x = last_human_pos_(0);
    human_pose.point.y = last_human_pos_(1);
    human_pose_pub_.publish(human_pose);
  }
}