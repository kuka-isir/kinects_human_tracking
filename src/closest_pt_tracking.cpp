#include <kinects_human_tracking/closest_pt_tracking.hpp>
/**
   Subscribe to a pointCloud and track the closest
   point to the robot's end-effector
 */

int main(int argc, char** argv){
  ros::init(argc, argv, "closest_pt_tracking");
  ros::NodeHandle nh, nh_priv("~");
  
  ROS_INFO("Initializing tracking...");  
  
  tf_listener_ = new tf::TransformListener();
   
  // Get params topics and frames names
  string kinect_topic_name, clusters_topic_name, out_topic_name;
  XmlRpc::XmlRpcValue clipping_rules_bounds;
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  params_loaded *= nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  params_loaded *= nh_priv.getParam("out_topic_name",out_topic_name);
  params_loaded *= nh_priv.getParam("voxel_size",voxel_size_);
  params_loaded *= nh_priv.getParam("min_cluster_size",min_cluster_size_);
  params_loaded *= nh_priv.getParam("kinect_noise",kinect_noise_);
  params_loaded *= nh_priv.getParam("kinect_noise_z",kinect_noise_z_);
  params_loaded *= nh_priv.getParam("process_noise",process_noise_);
  params_loaded *= nh_priv.getParam("minimum_height",minimum_height_);
  params_loaded *= nh_priv.getParam("max_tracking_jump",max_tracking_jump_);
  params_loaded *= nh_priv.getParam("clipping_rules",clipping_rules_bounds);
  params_loaded *= nh_priv.getParam("clustering_tolerance",clustering_tolerance_);
  params_loaded *= nh_priv.getParam("downsampling",downsampling_);
  params_loaded *= nh_priv.getParam("end_eff_frame",enf_eff_frame_);
  
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
  cluster_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  kinects_pc_->reserve(10000);
  cluster_cloud_->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_clustered_pub_ = nh.advertise<PointCloudSM>(clusters_topic_name, 1);
  cluster_pc_pub_ = nh.advertise<PointCloudSM>(out_topic_name, 1);
  cloud_mini_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>(kinect_topic_name+"/min_pt",1);
  cluster_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>(kinect_topic_name+"/tracking_state",1);
  track_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>(kinect_topic_name+"/closest_pt_tracking",1);
  dist_vect_pub_ = nh.advertise<geometry_msgs::Vector3>("sk_closest/vector_closest_frame",1);
  min_pub_ = nh.advertise<std_msgs::Float32>("minimum_distance",1);
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("velocity",1);
  ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>(kinect_topic_name, 1, callback);
  
  // Initialize Kalman filter
  Eigen::Matrix<float, 9, 1> x_k1;
  x_k1.fill(0.0);
  Eigen::Matrix<float, 9, 9> init_cov;
  init_cov.fill(0.0);
  kalman_.init(Eigen::Vector3f(kinect_noise_, kinect_noise_, kinect_noise_z_), Eigen::Vector3f(process_noise_ ,process_noise_,process_noise_), -1, x_k1, init_cov);
  
  // Initializing the tracking position at the origin
  last_pos_ = Eigen::Vector3f(0.0, 0.0, 0.0);
  
  ROS_INFO("Tracking ready !");
  
  last_observ_time_ = ros::Time(0.0);
  
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& kinect_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*kinect_pc_msg, *kinects_pc_);
  
  // Clip pointcloud using the rules defined in params
  pc_clipping(kinects_pc_, clipping_rules_ , kinects_pc_);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*kinects_pc_, *kinects_pc_, indices);
  
  // Downsampling the two pointClouds
  if(downsampling_)
    pc_downsampling(kinects_pc_, voxel_size_, kinects_pc_);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices = pc_clustering(kinects_pc_, min_cluster_size_, clustering_tolerance_ ,kinects_pc_);
  
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
      std::string tmp = boost::lexical_cast<std::string>(stats[i].max(2)-stats[i].min(2));
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
  if (cluster_indices.size()>0){
    get_closest_cluster_to_frame(kinects_pc_, cluster_indices, tf_listener_, enf_eff_frame_, cluster_cloud_, last_min_dist_, last_cluster_pt_);
    
    // Publish cluster's' pointCloud
    cluster_pc_pub_.publish(*cluster_cloud_);
    
    // Publish minimum point
    cloud_mini_pt_pub_.publish<geometry_msgs::PointStamped>(last_cluster_pt_);
    
    // Get pose observation from the stats
    Eigen::Vector3f obs;
    obs(0) = last_cluster_pt_.point.x;
    obs(1) = last_cluster_pt_.point.y;   
    obs(2) = last_cluster_pt_.point.z;   
    
    // If the new observation is too far from the previous one, reinitialize
    if(max_tracking_jump_>0){
      if ( (last_pos_-obs).norm() > max_tracking_jump_){
	Eigen::Matrix<float, 9, 1>  x_k1;
	x_k1.fill(0.);
	x_k1(0,0) = obs(0);
	x_k1(1,0) = obs(1);
	x_k1(2,0) = obs(2);
	kalman_.init(Eigen::Vector3f(kinect_noise_, kinect_noise_, kinect_noise_z_), Eigen::Vector3f(process_noise_ ,process_noise_, process_noise_), -1, x_k1);
	ROS_INFO("New pose too far. Reinitializing tracking!");
      }
    }
  
    // Feed the Kalman filter with the observation and get back the estimated state
    float delta_t;
    if (last_observ_time_.sec == 0)
      delta_t = -1;
    else
      delta_t = (ros::Time::now() - last_observ_time_).toSec();
    
    Eigen::Matrix<float, 9, 1> est;
    kalman_.estimate(obs, delta_t, est); 
    last_observ_time_ = ros::Time::now();
    
    // Save new estimated pose
    last_pos_(0) = est(0);
    last_pos_(1) = est(1);    
    last_pos_(2) = est(2);    
    
    // Visualize pose and speed
    visualize_state(est, cluster_state_pub_);  
    
    // Publish minimum distance and speed
    std_msgs::Float32 float32_msg;
    float32_msg.data = last_min_dist_;
    min_pub_.publish(std_msgs::Float32(float32_msg));
    geometry_msgs::Twist twist;
    twist.linear.x = est(3);
    twist.linear.y = est(4);
    twist.linear.z = est(5);
    vel_pub_.publish(twist);
    
    // Publish vector between point and end-effector
    tf::StampedTransform end_eff_transform;
    try{
      tf_listener_->waitForTransform(kinects_pc_->header.frame_id, enf_eff_frame_, ros::Time(0.0), ros::Duration(1.0));
      tf_listener_->lookupTransform(kinects_pc_->header.frame_id, enf_eff_frame_, ros::Time(0.0), end_eff_transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
    
    geometry_msgs::Vector3 dist_vect;
    dist_vect.x = end_eff_transform.getOrigin().getX() - est(0);
    dist_vect.y = end_eff_transform.getOrigin().getY() - est(1);
    dist_vect.z = end_eff_transform.getOrigin().getZ() - est(2);
    dist_vect_pub_.publish(dist_vect);
    
    geometry_msgs::PointStamped closest_pt;
    closest_pt.header.frame_id = kinects_pc_->header.frame_id;
    closest_pt.point.x = est(0);
    closest_pt.point.y = est(1);
    closest_pt.point.z = est(2);
    track_pt_pub_.publish(closest_pt);
    
  }
}

void visualize_state (Eigen::Matrix<float, 9, 1> state, ros::Publisher state_pub){
  
  visualization_msgs::MarkerArray markers_arr;
  visualization_msgs::Marker velx_marker, vely_marker, velz_marker, vel_marker;
  markers_arr.markers.clear();
  
  tf::Vector3 axis_vector, right_vector;
  tf::Vector3 x_axis(1,0,0);
  tf::Quaternion quat;
  
  // Marker for velocity on x
  velx_marker.header.frame_id = kinects_pc_->header.frame_id;
  velx_marker.header.stamp = ros::Time::now();
  velx_marker.id = 200;
  velx_marker.ns = "velx";
  velx_marker.type = visualization_msgs::Marker::ARROW;
  velx_marker.action = visualization_msgs::Marker::ADD;
  velx_marker.pose.position.x = state(0);
  velx_marker.pose.position.y = state(1);
  velx_marker.pose.position.z = state(2);
  velx_marker.scale.y = 0.03;
  velx_marker.scale.z = 0.03;
  velx_marker.scale.x = abs(state(3));
  velx_marker.color.r = 1.0f;
  velx_marker.color.g = 0.0f;
  velx_marker.color.b = 0.0f;
  velx_marker.color.a = 1.0f;
  velx_marker.lifetime = ros::Duration();

  if (state(3)<=0){    
    quat.setEuler(M_PI, 0, 0);
    velx_marker.pose.orientation.x = quat.getX(); 
    velx_marker.pose.orientation.y = quat.getY();
    velx_marker.pose.orientation.z = quat.getZ();
    velx_marker.pose.orientation.w = quat.getW();
    
  }
  markers_arr.markers.push_back(velx_marker);
  
  // Marker for velocity on y
  vely_marker = velx_marker;
  vely_marker.id = 201;
  vely_marker.ns = "vely";
  vely_marker.scale.x = abs(state(4));
  vely_marker.color.r = 0.0f;
  vely_marker.color.g = 1.0f;
  vely_marker.color.b = 0.0f;
  vely_marker.lifetime = ros::Duration();
  
  axis_vector = tf::Vector3(0,state(4),0);
  right_vector = axis_vector.cross(x_axis);
  right_vector.normalize();
  quat = tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(x_axis)));
  quat.normalize();
  vely_marker.pose.orientation.x = quat.getX(); 
  vely_marker.pose.orientation.y = quat.getY();
  vely_marker.pose.orientation.z = quat.getZ();
  vely_marker.pose.orientation.w = quat.getW();
  
  markers_arr.markers.push_back(vely_marker);
  
  // Marker for velocity on z
  velz_marker = vely_marker;
  velz_marker.id = 202;
  velz_marker.ns = "velz";
  velz_marker.scale.x = abs(state(5));
  velz_marker.color.r = 0.0f;
  velz_marker.color.g = 0.0f;
  velz_marker.color.b = 1.0f;
  velz_marker.lifetime = ros::Duration();
  
  axis_vector = tf::Vector3(0,0,state(5));
  right_vector = axis_vector.cross(x_axis);
  right_vector.normalize();
  quat = tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(x_axis)));
  quat.normalize();
  velz_marker.pose.orientation.x = quat.getX(); 
  velz_marker.pose.orientation.y = quat.getY();
  velz_marker.pose.orientation.z = quat.getZ();
  velz_marker.pose.orientation.w = quat.getW();
  
  markers_arr.markers.push_back(velz_marker);
  
  // Marker for global velocity
  vel_marker = velz_marker;
  vel_marker.id = 204;
  vel_marker.ns = "vel";
  vel_marker.scale.x = sqrt(pow(state(3),2)+pow(state(4),2)+pow(state(5),2));
  vel_marker.color.r = 1.0f;
  vel_marker.color.g = 0.0f;
  vel_marker.color.b = 1.0f;
  vel_marker.lifetime = ros::Duration();
  
  axis_vector = tf::Vector3(state(3),state(4),state(5));
  right_vector = axis_vector.cross(x_axis);
  right_vector.normalize();
  quat = tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(x_axis)));
  quat.normalize();
  vel_marker.pose.orientation.x = quat.getX(); 
  vel_marker.pose.orientation.y = quat.getY();
  vel_marker.pose.orientation.z = quat.getZ();
  vel_marker.pose.orientation.w = quat.getW();
  markers_arr.markers.push_back(vel_marker);
  
  
  state_pub.publish(markers_arr);
  
}