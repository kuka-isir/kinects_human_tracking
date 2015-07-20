#include <kinects_human_tracking/kinect_human_tracking.hpp>
/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then track him using a Kalman filter
 */

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_human_tracking");
  ros::NodeHandle nh, nh_priv("~");
  
  ROS_INFO("Initializing human tracking...");  
   
  // Get params topics and frames names
  string kinect_topic_name, robot_topic_name, clusters_topic_name, out_topic_name;
  XmlRpc::XmlRpcValue clipping_rules_bounds;
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  params_loaded *= nh_priv.getParam("robot_topic_name",robot_topic_name);
  params_loaded *= nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  params_loaded *= nh_priv.getParam("out_topic_name",out_topic_name);
  params_loaded *= nh_priv.getParam("voxel_size",voxel_size_);
  params_loaded *= nh_priv.getParam("min_cluster_size",min_cluster_size_);
  params_loaded *= nh_priv.getParam("kinect_noise",kinect_noise_);
  params_loaded *= nh_priv.getParam("process_noise",process_noise_);
  params_loaded *= nh_priv.getParam("minimum_height",minimum_height_);
  params_loaded *= nh_priv.getParam("max_tracking_jump",max_tracking_jump_);
  params_loaded *= nh_priv.getParam("clipping_rules",clipping_rules_bounds);
  
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    return -1;
  }
  
  if (clipping_rules_bounds.size()>0){
    if (clipping_rules_bounds.size()%3)
      ROS_ERROR("Problem in defining the clipping rules.\n Use the following format:\n [x, GT, 1.0, y, LT, 3.1, ...]");
    else{
      clipping_rules_.resize(clipping_rules_bounds.size()/3);
      ClippingRule new_rule;
      for(int i=0; i<clipping_rules_bounds.size()/3;i++){
	new_rule.axis = static_cast<string>(clipping_rules_bounds[i*3]);
	new_rule.op = static_cast<string>(clipping_rules_bounds[i*3+1]);
	new_rule.val = static_cast<double>(clipping_rules_bounds[i*3+2]);
	clipping_rules_.at(i) = new_rule;
      }
    }
    ROS_INFO("%d clipping rules loaded", static_cast<int>(clipping_rules_.size()));
  }
  
  // Initialize PointClouds
  kinects_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  robot_pc_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >(new pcl::PointCloud<pcl::PointXYZI>);
  human_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  kinects_pc_->reserve(10000);
  robot_pc_->reserve(10000);
  human_cloud_->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_clustered_pub_ = nh.advertise<PointCloudSM>(clusters_topic_name, 1);
  human_pc_pub_ = nh.advertise<PointCloudSM>(out_topic_name, 1);
  cloud_mini_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>(kinect_topic_name+"/min_human_pt",1);
  dist_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>(kinect_topic_name+"/min_robot_pt",1);
  human_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>(kinect_topic_name+"/human_state",1);
  
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
  
  ROS_INFO("Human tracking ready !");
  
  last_observ_time_ = ros::Time(0.0);
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*human_pc_msg, *kinects_pc_);
  pcl::fromROSMsg(*robot_pc_msg, *robot_pc_);  
  
  for (int i=0; i<robot_pc_->points.size();i++)
    cout<<"Label : "<< robot_pc_->points[i].intensity <<endl;
  
  sleep(10000);
  return;
  
  // Clip pointcloud using the rules defined in params
  pc_clipping(kinects_pc_, clipping_rules_ , kinects_pc_);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*kinects_pc_, *kinects_pc_, indices);
  
  // Downsampling the two pointClouds
  pc_downsampling(kinects_pc_, voxel_size_, kinects_pc_);
  pc_downsampling(robot_pc_, voxel_size_, robot_pc_);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices = pc_clustering(kinects_pc_, 2*voxel_size_ ,kinects_pc_);
  
  // Gives each cluster a random color
  for(int i=0; i<cluster_indices.size();i++){
    uint8_t r(rand()%255), g(rand()%255), b(rand()%255);
    for(int j=0; j<cluster_indices[i].indices.size();j++){
      kinects_pc_->points[cluster_indices[i].indices[j]].r = r;
      kinects_pc_->points[cluster_indices[i].indices[j]].g = g;
      kinects_pc_->points[cluster_indices[i].indices[j]].b = b;
    }
  }
  
  // Publishing clusters
  pc_clustered_pub_.publish(*kinects_pc_);
    
  if(minimum_height_ >0){
    // Getting heights for all clusters
    std::vector<double> cluster_heights; 
    vector<ClusterStats> stats = get_clusters_stats (kinects_pc_ , cluster_indices);
    for(int i=0; i<cluster_indices.size();i++){
      std::string tmp = boost::lexical_cast<std::string>(stats[i].max-stats[i].min);
      double cluster_height = (double)atof(tmp.c_str());
      cluster_heights.push_back(cluster_height);
    }

    // Selecting only the clusters with the minimum_height   
    std::vector<pcl::PointIndices> tmp_cluster_indices;
    for(int i=0; i<cluster_indices.size(); i++){
      if (cluster_heights[i]>=minimum_height_)
	tmp_cluster_indices.push_back(cluster_indices[i]);
    }
    cluster_indices = tmp_cluster_indices;
  }
   
  // Get closest cluster to the robot
  get_closest_cluster_to_robot(kinects_pc_, cluster_indices, robot_pc_, human_cloud_, last_min_dist_, last_human_pt_, last_robot_pt_);
  
  if (cluster_indices.size()>0){
    // Publish human pointCloud
    human_pc_pub_.publish(*human_cloud_);
    
    // Publish minimum points 
    cloud_mini_pt_pub_.publish<geometry_msgs::PointStamped>(last_human_pt_);
    dist_pt_pub_.publish<geometry_msgs::PointStamped>(last_robot_pt_);
    
    // Get stats on human's pointCloud
    ClusterStats human_stats = get_cluster_stats(human_cloud_);
    
    // Get pose observation from the stats
    Eigen::Vector2f obs;
    obs(0) = human_stats.median(0);
    obs(1) = human_stats.median(1);
    Eigen::Matrix<float, 6, 1>  x_k1;
    x_k1.fill(0.);
    x_k1(0,0) = obs(0);
    x_k1(1,0) = obs(1);
    
    // If the new observation is too far from the previous one, reinitialize
    if ( (last_human_pos_-obs).norm() > max_tracking_jump_){
      kalman_.init(Eigen::Vector2f(kinect_noise_, process_noise_), Eigen::Vector2f(process_noise_ ,process_noise_), -1, x_k1);
      ROS_INFO("New human pose to far. Reinitializing tracking!");
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
    
    // Visualize human pose and speed
    visualize_state(est, human_stats, human_state_pub_);  
  }
}

void visualize_state (Eigen::Matrix<float, 6, 1> state, ClusterStats stats, ros::Publisher state_pub){
  
  visualization_msgs::MarkerArray markers_arr;
  visualization_msgs::Marker inn_cyl_marker, out_cyl_marker, vel1_marker, vel2_marker, vel3_marker;
  markers_arr.markers.clear();
  
  //Inner-cylinder marker
  inn_cyl_marker.header.frame_id = "world";
  inn_cyl_marker.header.stamp = last_observ_time_;
  inn_cyl_marker.id = 0;
  inn_cyl_marker.type = visualization_msgs::Marker::CYLINDER;
  inn_cyl_marker.action = visualization_msgs::Marker::ADD;
  inn_cyl_marker.pose.position.x = state(0);
  inn_cyl_marker.pose.position.y = state(1);
  inn_cyl_marker.pose.position.z = stats.max(2)/2.0;
  inn_cyl_marker.scale.x = 2.0*sqrt(stats.var(0));
  inn_cyl_marker.scale.y = 2.0*sqrt(stats.var(1));
  inn_cyl_marker.scale.z = stats.max(2);
  inn_cyl_marker.color.r = 1.0f;
  inn_cyl_marker.color.g = 0.0f;
  inn_cyl_marker.color.b = 0.0f;
  inn_cyl_marker.color.a = 0.5f;
  inn_cyl_marker.lifetime = ros::Duration();
  markers_arr.markers.push_back(inn_cyl_marker);
  
  // Outer-cylinder marker
  out_cyl_marker = inn_cyl_marker;
  out_cyl_marker.id = 1;
  out_cyl_marker.scale.x = max(fabs(stats.median(0)-stats.min(0)),fabs(stats.median(0)-stats.max(0))); 
  out_cyl_marker.scale.y = max(fabs(stats.median(1)-stats.min(1)),fabs(stats.median(1)-stats.max(1)));
  markers_arr.markers.push_back(out_cyl_marker);
 
  double delta_t = 0.33333;
  double vel = sqrt(state(2)*state(2)+state(3)*state(3));
  double vel_scale = max(out_cyl_marker.scale.x, out_cyl_marker.scale.y);
  double marker_scale = delta_t*vel;
  
  // Vel1 marker
  vel1_marker = inn_cyl_marker;
  vel1_marker.id = 2;
  vel1_marker.pose.position.x = state(0) + delta_t * state(2);
  vel1_marker.pose.position.y = state(1) + delta_t * state(3);;
  vel1_marker.pose.position.z = 0.0;
  vel1_marker.scale.x = vel_scale + delta_t * vel+ marker_scale;
  vel1_marker.scale.y = vel_scale + delta_t * vel+ marker_scale;
  vel1_marker.scale.z = 0.01;
  vel1_marker.color.r = 0.75f;
  vel1_marker.color.g = 1.0f;
  vel1_marker.color.b = 0.0f;
  vel1_marker.color.a = 1.0f;
  markers_arr.markers.push_back(vel1_marker);
  
  // Vel2 marker
  vel2_marker = vel1_marker;
  vel2_marker.id = 3;
  vel2_marker.pose.position.x += delta_t * state(2) ;
  vel2_marker.pose.position.y += delta_t * state(3) ;
  vel2_marker.scale.x += marker_scale;
  vel2_marker.scale.y += marker_scale;
  vel2_marker.color.a = 0.5f;
  markers_arr.markers.push_back(vel2_marker);
  
  // Vel3 marker
  vel3_marker = vel1_marker;
  vel3_marker.id = 4;
  vel3_marker.pose.position.x += 2.0 * delta_t * state(2) ;
  vel3_marker.pose.position.y += 2.0 * delta_t * state(3) ;
  vel3_marker.scale.x += 2.0 * marker_scale;
  vel3_marker.scale.y += 2.0 * marker_scale;
  vel3_marker.color.a = 0.25f;  
  markers_arr.markers.push_back(vel3_marker);
  
  state_pub.publish(markers_arr);
  
}