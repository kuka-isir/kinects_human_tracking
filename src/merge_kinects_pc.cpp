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
// Read topics from params, and set default ones
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
string front_frame, back_frame, new_frame;
ros::Publisher pc_pub;
PointCloudSM::Ptr first_pcl_pc1, second_pcl_pc1, transformed_first_pc, transformed_second_pc, merged_pc;

int main(int argc, char** argv){
  ros::init(argc, argv, "merge_kinects_pc");
  ros::NodeHandle nh;
  
  first_frame = true;
  
  // Initialize shared pointers
  first_pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  second_pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  transformed_first_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  transformed_second_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  merged_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Initialize PointClouds
  PointCloudSM::Ptr transformed_first_pc(new PointCloudSM), transformed_second_pc(new PointCloudSM), merged_pc(new PointCloudSM);
  
  // Get subscribed topics names
  string bg_back_topic = "background_sub_back/pc1_out";
  string bg_front_topic = "background_sub_front/pc1_out";
  new_frame = "table_link";
  
  // Ros Subscribers and Publishers
  pc_pub = nh.advertise<PointCloudSM>("/kinect_both/depth_registered/points", 1);
  message_filters::Subscriber<PCMsg> front_sub(nh, bg_front_topic, 1);
  message_filters::Subscriber<PCMsg> back_sub(nh, bg_back_topic, 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(4), back_sub, front_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& first_msg_pc2_, const PCMsg::ConstPtr& second_msg_pc2 ){
  
  // conversionns to pcl::PointCloud
  pcl::PCLPointCloud2 first_pcl_pc2, second_pcl_pc2 ;
  pcl_conversions::toPCL(*first_msg_pc2_, first_pcl_pc2);
  pcl::fromPCLPointCloud2(first_pcl_pc2, *first_pcl_pc1);
  pcl_conversions::toPCL(*second_msg_pc2, second_pcl_pc2);
  pcl::fromPCLPointCloud2(second_pcl_pc2, *second_pcl_pc1);

  string first_pc_frame = first_pcl_pc1->header.frame_id;
  string second_pc_frame = second_pcl_pc1->header.frame_id;

  // get transforms the first time
  if(first_frame){
    get_transforms(first_pc_frame, second_pc_frame, first_transform, second_transform);
    first_frame=false;
  }

  // transform points
  pcl::transformPointCloud(*second_pcl_pc1, *transformed_second_pc, first_transform.translation, first_transform.rotation);
  pcl::transformPointCloud(*first_pcl_pc1, *transformed_first_pc, second_transform.translation, second_transform.rotation);

  transformed_second_pc->header.frame_id = new_frame;
  transformed_first_pc->header.frame_id = new_frame;

  if(COLOR_DIFF){ //change PC colors
    for(size_t i=0; i<transformed_first_pc->size(); ++i){
      transformed_first_pc->at(i).r = uint8_t(0);
      transformed_first_pc->at(i).g = uint8_t(255);	  
      transformed_first_pc->at(i).b = uint8_t(0);
    }
    for(size_t i=0; i<transformed_second_pc->size(); ++i){
      transformed_second_pc->at(i).r = uint8_t(255);
      transformed_second_pc->at(i).g = uint8_t(0);	  
      transformed_second_pc->at(i).b = uint8_t(0);
    }
  }

  *merged_pc = *transformed_first_pc;
  *merged_pc += *transformed_second_pc;
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
	first_trans_listener.waitForTransform(new_frame, first_pc_frame, ros::Time(0), ros::Duration(10.0));
	first_trans_listener.lookupTransform(new_frame, first_pc_frame, ros::Time(0), stampedTransform);
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
	second_trans_listener.waitForTransform(new_frame, second_pc_frame, ros::Time(0),ros::Duration(10.0));
	second_trans_listener.lookupTransform(new_frame, second_pc_frame, ros::Time(0), stampedTransform);
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