#include <kinects_human_tracking/object_detection.hpp>
/**
   Subscribe to a pointCloud and detects the objects
   on the table
 */

int main(int argc, char** argv){
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh, nh_priv("~");
  
  ROS_INFO("Initializing detection...");  
  
    // Get params topics and frames names
  string kinect_topic_name, clusters_topic_name, out_topic_name;
  XmlRpc::XmlRpcValue clipping_rules_bounds;
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("kinect_topic_name",kinect_topic_name);  
  params_loaded *= nh_priv.getParam("clusters_topic_name",clusters_topic_name);
//   params_loaded *= nh_priv.getParam("out_topic_name",out_topic_name);
  params_loaded *= nh_priv.getParam("voxel_size",voxel_size_);
  params_loaded *= nh_priv.getParam("min_cluster_size",min_cluster_size_);
  params_loaded *= nh_priv.getParam("minimum_height",minimum_height_);
  params_loaded *= nh_priv.getParam("clipping_rules",clipping_rules_bounds);
  params_loaded *= nh_priv.getParam("clustering_tolerance",clustering_tolerance_);
  params_loaded *= nh_priv.getParam("downsampling",downsampling_);
  
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    ROS_INFO_STREAM("kinect topic: "<<kinect_topic_name);
    ROS_INFO_STREAM("clusters topic: "<<clusters_topic_name);
    ROS_INFO_STREAM("voxel size: "<<voxel_size_);
    ROS_INFO_STREAM("min cluster size: "<<min_cluster_size_);
    ROS_INFO_STREAM("min height: "<<minimum_height_);
    ROS_INFO_STREAM("clipping rules: "<<clipping_rules_bounds);
    ROS_INFO_STREAM("clustering tolerance: "<<clustering_tolerance_);
    ROS_INFO_STREAM("downsampling: "<< downsampling_ ? "true" : "false" );
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
  ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>(kinect_topic_name, 1, callback);
  cluster_pc_pub_ = nh.advertise<PCMsg>(clusters_topic_name, 1);
  cubes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/object_detection/markers", 1);
  
  ROS_INFO("Detection running !"); 
  
  first_frame_ = true;
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& kinect_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*kinect_pc_msg, *kinects_pc_);
  
  if(first_frame_){
      // get transform between the pc frame and the output frame
      get_transform(kinects_pc_->header.frame_id, "base_link", pcl_transform_);
      first_frame_ = false;
  }
  
  // transform the pc to the output frame
  pcl::transformPointCloud(*kinects_pc_, *kinects_pc_, pcl_transform_.translation, pcl_transform_.rotation);
  kinects_pc_->header.frame_id = "base_link";
  
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
  
//   // Gives each cluster a random color
//   for(int i=0; i<cluster_indices.size();i++){
//     uint8_t r(rand()%255), g(rand()%255), b(rand()%255);
//     for(int j=0; j<cluster_indices[i].indices.size();j++){
//       kinects_pc_->points[cluster_indices[i].indices[j]].r = r;
//       kinects_pc_->points[cluster_indices[i].indices[j]].g = g;
//       kinects_pc_->points[cluster_indices[i].indices[j]].b = b;
//     }
//   }
  
  // Publishing clusters
  cluster_pc_pub_.publish(*kinects_pc_);
  

  // Getting heights for all clusters
  std::vector<double> cluster_heights; 
  vector<ClusterStats> stats = get_clusters_stats (kinects_pc_,cluster_indices);
  
//   for(int i=0; i<cluster_indices.size();i++){
//     std::string tmp = 
//     boost::lexical_cast<std::string>(stats[i].max(2)-stats[i].min(2));
//     double cluster_height = (double)atof(tmp.c_str());
//     cluster_heights.push_back(cluster_height);
//   }
  
  
  
  // Create a bounding box for each cluster
  visualization_msgs::MarkerArray cubes;
  visualization_msgs::Marker cube;
  cube.header.frame_id = "base_link";
  cube.header.stamp = ros::Time::now();
  cube.action = visualization_msgs::Marker::ADD;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.color.r = 0.0f;
  cube.color.g = 0.0f;
  cube.color.b = 1.0f;
  cube.color.a = 0.3f;
  cube.lifetime = ros::Duration(0.1);
  
  for(int i=0; i<cluster_indices.size();i++){
    cube.id = 1000 + i;
    cube.ns = "cube"+boost::lexical_cast<std::string>(i);
    
    cube.pose.position.x = stats[i].mean(0);
    cube.pose.position.y = stats[i].mean(1);
    cube.pose.position.z = stats[i].mean(2);
    cube.pose.orientation.x = 0.0;
    cube.pose.orientation.y = 0.0;
    cube.pose.orientation.z = 0.0;
    cube.pose.orientation.w = 1.0;
    cube.scale.x = stats[i].max(0)-stats[i].min(0);
    cube.scale.y = stats[i].max(1)-stats[i].min(1);
    cube.scale.z = stats[i].max(2)-stats[i].min(2);
    
    cubes.markers.push_back(cube);
  }
  
  cubes_pub_.publish(cubes);

//   if(minimum_height_ >0){
//     // Getting heights for all clusters
//     std::vector<double> cluster_heights; 
//     vector<ClusterStats> stats = get_clusters_stats (kinects_pc_ , cluster_indices);
//     for(int i=0; i<cluster_indices.size();i++){
//       std::string tmp = boost::lexical_cast<std::string>(stats[i].max(2)-stats[i].min(2));
//       double cluster_height = (double)atof(tmp.c_str());
//       cluster_heights.push_back(cluster_height);
//     }
// 
//     // Selecting only the clusters with the minimum_height   
//     std::vector<pcl::PointIndices> tmp_cluster_indices;
//     for(int i=0; i<cluster_indices.size(); i++){
//       if (cluster_heights[i]>=minimum_height_)
// 	tmp_cluster_indices.push_back(cluster_indices[i]);
//     }
//     cluster_indices = tmp_cluster_indices;
//   }
   
}

void get_transform(string in_frame, string out_frame, pclTransform &pc_transform){
   
  tf::TransformListener trans_listener;
  tf::StampedTransform stampedTransform;
  
  try{
    trans_listener.waitForTransform(out_frame, in_frame, ros::Time(0), ros::Duration(10.0));
    trans_listener.lookupTransform(out_frame, in_frame, ros::Time(0), stampedTransform);
  }
  catch(tf::TransformException &ex){
    cout << ex.what() << endl;
    ROS_ERROR("%s", ex.what());
  }
  tf::vectorTFToEigen(stampedTransform.getOrigin(), pc_transform.translation);      
  tf::quaternionTFToEigen(stampedTransform.getRotation(), pc_transform.rotation);

  return;
}