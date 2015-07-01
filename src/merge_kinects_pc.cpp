#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <opencv2/opencv.hpp>
#include <boost/timer.hpp>

//ros-includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/**
   Subscribe to two point clouds, transform into 
   one frame. Merge them and then publish as a
   new Point Cloud.
**/

/******************* TODO **********************************************/
// Convert to a n-pointclouds merge with std::vector


#define COLOR_DIFF false //specifies if the two point clouds to be published with different colors
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg> MySyncPolicy;

struct pclTransform{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};

void callback(const PCMsg::ConstPtr& front_pc, const PCMsg::ConstPtr& back_pc );
void get_transforms( string first_pc_frame, string second_pc_frame, pclTransform &first_transform, pclTransform &second_transform);

pclTransform first_transform, second_transform;
bool first_frame;
string first_topic_name, second_topic_name, out_frame, out_topic_name;
ros::Publisher pc_pub;
PointCloudSM::Ptr first_pcl_pc1, second_pcl_pc1, merged_pc;

int main(int argc, char** argv){
  ros::init(argc, argv, "merge_kinects_pc");
  ros::NodeHandle nh, nh_priv("~");
  
  first_frame = true;
  
  // Get params topics and frames names
  nh_priv.getParam("first_topic_name", first_topic_name);
  nh_priv.getParam("second_topic_name", second_topic_name);
  nh_priv.getParam("out_frame", out_frame);
  nh_priv.getParam("out_topic_name", out_topic_name);
  
  // Initialize shared pointers
  first_pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  second_pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  merged_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for pointclouds
  first_pcl_pc1->reserve(10000);
  second_pcl_pc1->reserve(10000);
  merged_pc->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_pub = nh.advertise<PointCloudSM>(out_topic_name, 1);
  message_filters::Subscriber<PCMsg> first_sub(nh, first_topic_name, 1);
  message_filters::Subscriber<PCMsg> second_sub(nh, second_topic_name, 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), first_sub, second_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& first_msg_pc2_, const PCMsg::ConstPtr& second_msg_pc2 ){
  
  // conversionns to pcl::PointCloud 
  pcl::fromROSMsg(*first_msg_pc2_, *first_pcl_pc1);
  pcl::fromROSMsg(*second_msg_pc2, *second_pcl_pc1);

  string first_pc_frame = first_pcl_pc1->header.frame_id;
  string second_pc_frame = second_pcl_pc1->header.frame_id;

  // get transforms the first time
  if(first_frame){
    get_transforms(first_pc_frame, second_pc_frame, first_transform, second_transform);
    first_frame=false;
  }

  // transform points
  pcl::transformPointCloud(*second_pcl_pc1, *second_pcl_pc1, first_transform.translation, first_transform.rotation);
  pcl::transformPointCloud(*first_pcl_pc1, *first_pcl_pc1, second_transform.translation, second_transform.rotation);

  second_pcl_pc1->header.frame_id = out_frame;
  first_pcl_pc1->header.frame_id = out_frame;

  if(COLOR_DIFF){ //change PC colors
    for(size_t i=0; i<first_pcl_pc1->size(); ++i){
      first_pcl_pc1->at(i).r = uint8_t(0);
      first_pcl_pc1->at(i).g = uint8_t(255);	  
      first_pcl_pc1->at(i).b = uint8_t(0);
    }
    for(size_t i=0; i<second_pcl_pc1->size(); ++i){
      second_pcl_pc1->at(i).r = uint8_t(255);
      second_pcl_pc1->at(i).g = uint8_t(0);	  
      second_pcl_pc1->at(i).b = uint8_t(0);
    }
  }

  *merged_pc = *first_pcl_pc1;
  *merged_pc += *second_pcl_pc1;
  pc_pub.publish(*merged_pc);
}

void get_transforms(string first_pc_frame, string second_pc_frame, pclTransform &first_transform, pclTransform &second_transform){
 
  //Listeners for both transformations
  tf::TransformListener first_trans_listener;
  tf::TransformListener second_trans_listener;

  //listen for transform until one is gotten
  //since its static, don't look for transform afterwards
  bool found_first_t=false, found_second_t=false;
  tf::StampedTransform stampedTransform;
  
  while (!(found_first_t && found_second_t)){
    if(!found_first_t){
      try{
	first_trans_listener.waitForTransform(out_frame, first_pc_frame, ros::Time(0), ros::Duration(10.0));
	first_trans_listener.lookupTransform(out_frame, first_pc_frame, ros::Time(0), stampedTransform);
      }
      catch(tf::TransformException &ex){
	cout << ex.what() << endl;
	ROS_ERROR("%s", ex.what());
	continue;
      }
      //Store transform
      found_first_t=true;
      tf::vectorTFToEigen(stampedTransform.getOrigin(), second_transform.translation);      
      tf::quaternionTFToEigen(stampedTransform.getRotation(), second_transform.rotation);
      
    }
    
    if(!found_second_t){
      try{
	second_trans_listener.waitForTransform(out_frame, second_pc_frame, ros::Time(0),ros::Duration(10.0));
	second_trans_listener.lookupTransform(out_frame, second_pc_frame, ros::Time(0), stampedTransform);
      }
      catch(tf::TransformException &ex){
	cout << ex.what() << endl;
	ROS_ERROR("%s", ex.what());
	continue;
      }
      //Store transform
      found_second_t=true;
      tf::vectorTFToEigen(stampedTransform.getOrigin(), first_transform.translation);      
      tf::quaternionTFToEigen(stampedTransform.getRotation(), first_transform.rotation);
    }
  }

  return;
}