#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <boost/timer.hpp>

//ros-includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>

/**
   Subscribe to a pointCloud and transforms it
   to the right frame
**/

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;

struct pclTransform{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};

void callback(const PCMsg::ConstPtr& msg_pc2);
void get_transform(string pc_frame);

pclTransform transform_;
bool first_frame_;
string in_topic_name_, out_frame_, out_topic_name_;
ros::Publisher pc_pub_;
PointCloudSM::Ptr pcl_pc1_;

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_pc_transform");
  ros::NodeHandle nh, nh_priv("~");
  first_frame_ = true;
  
  ROS_INFO("Initializing pointCloud transformation...");
  
  // Get params topics and frames names
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("in_topic_name", in_topic_name_);
  params_loaded *= nh_priv.getParam("out_frame", out_frame_);
  params_loaded *= nh_priv.getParam("out_topic_name", out_topic_name_);
  
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    return -1;
  }
  
  // Initialize shared pointers
  pcl_pc1_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for pointclouds
  pcl_pc1_->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_pub_ = nh.advertise<PointCloudSM>(out_topic_name_, 1);
  ros::Subscriber pc_sub = nh.subscribe<PCMsg>(in_topic_name_, 1, callback);
  
  ROS_INFO("PointCloud transformation running!");
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& msg_pc2){
  
  // conversionns to pcl::PointCloud 
  pcl::fromROSMsg(*msg_pc2, *pcl_pc1_);

  // get transforms the first time
  if(first_frame_){
    get_transform(pcl_pc1_->header.frame_id);
    first_frame_ = false;
  }

  // Transform pointCloud
  pcl::transformPointCloud(*pcl_pc1_, *pcl_pc1_, transform_.translation, transform_.rotation);
  pcl_pc1_->header.frame_id = out_frame_;

  pc_pub_.publish(*pcl_pc1_);
}

void get_transform(string pc_frame){
 
  //Listeners for both transformations
  tf::TransformListener trans_listener;

  //listen for transform until we get one
  bool found_t = false;
  tf::StampedTransform stampedTransform;
  
  while (!found_t){
    try{
      trans_listener.waitForTransform(out_frame_, pc_frame, ros::Time(0), ros::Duration(10.0));
      trans_listener.lookupTransform(out_frame_, pc_frame, ros::Time(0), stampedTransform);
    }
    catch(tf::TransformException &ex){
      cout << ex.what() << endl;
      ROS_ERROR("%s", ex.what());
      continue;
    }
    //Store transform
    found_t = true;
    tf::vectorTFToEigen(stampedTransform.getOrigin(), transform_.translation);      
    tf::quaternionTFToEigen(stampedTransform.getRotation(), transform_.rotation);  
  }
  return;
  
}