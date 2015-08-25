#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <boost/timer.hpp>
#include <boost/graph/graph_concepts.hpp>

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
   Subscribe to 1 to 5 pointclouds, transform into 
   one frame. Merge them and then publish as a
   new Point Cloud.
**/

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg> MySyncPolicy2;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg> MySyncPolicy3;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg, PCMsg> MySyncPolicy4;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg, PCMsg, PCMsg> MySyncPolicy5;

struct pclTransform{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};

// void callback(const PCMsg::ConstPtr& front_pc, const PCMsg::ConstPtr& back_pc );
void callback5(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2 );
void callback4(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2);
void callback3(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2 );
void callback2(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2);
void callback(const PCMsg::ConstPtr& first_msg_pc2);
void merge_pcs(vector<PCMsg::ConstPtr> pcs_msg_pc2);

pclTransform first_transform, second_transform;
bool first_frame;
string out_frame, out_topic_name;
vector<string> in_topic_names;
ros::Publisher pc_pub;
vector<PointCloudSM::Ptr> pcs;
vector<PCMsg::ConstPtr> pcs_msg_pc2;
PointCloudSM::Ptr merged_pc;

int main(int argc, char** argv){
  ros::init(argc, argv, "merge_kinects_pc");
  ros::NodeHandle nh, nh_priv("~");
  
  first_frame = true;
  
  // Get params topics and frames names
  int nb_topics = 0;
  bool done_loading;
  string topic_name, temp;
  cout << "Merging : " << topic_name <<endl;
  do{
    topic_name = "topic_name"+ boost::lexical_cast<string>(nb_topics+1);    
    done_loading = (!nh_priv.getParam(topic_name, temp)) || (temp=="") ;
    
    if(!done_loading){
      in_topic_names.push_back(temp);
      cout << temp <<endl;
      nb_topics++;
    }
    
  }while(!done_loading);
  cout << endl;
  cout << "Merging "<<nb_topics<<" topics"<<endl<<endl;
  
  nh_priv.getParam("out_frame", out_frame);
  nh_priv.getParam("out_topic_name", out_topic_name);
  
  // Initialize shared pointers and reserve memory
  for (int i=0; i<nb_topics;i++){
    pcs.push_back(boost::shared_ptr<PointCloudSM>(new PointCloudSM));
    pcs[i]->reserve(10000);
  }
  merged_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM); 
  merged_pc->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_pub = nh.advertise<PointCloudSM>(out_topic_name, 1);
  
  vector<boost::shared_ptr<message_filters::Subscriber<PCMsg> > > subs(nb_topics);
  for (int i=0; i<nb_topics; i++)
    subs[i] = boost::shared_ptr<message_filters::Subscriber<PCMsg> >(new message_filters::Subscriber<PCMsg>(nh, in_topic_names[i],1));
  
  switch( nb_topics ){
    case 0: 
      ROS_ERROR("No topics to merge provided");
      return 1; break;
    case 1:{
      ros::Subscriber pc_sub = nh.subscribe<PCMsg>(in_topic_names[0], 1, callback);
      ros::spin();
      break;
    }
    case 2:{ 
      message_filters::Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10), *subs[0], *subs[1]);
      sync2.registerCallback(boost::bind(&callback2, _1, _2));
      cout <<"callback2"<<endl;
      ros::spin();
      break;
    }
    case 3:{
      message_filters::Synchronizer<MySyncPolicy3> sync3(MySyncPolicy3(10), *subs[0], *subs[1], *subs[2]);
      sync3.registerCallback(boost::bind(&callback3, _1, _2, _3));
      ros::spin();
      break;
    }
    case 4:{
      message_filters::Synchronizer<MySyncPolicy4> sync4(MySyncPolicy4(10), *subs[0], *subs[1], *subs[2], *subs[3]);
      sync4.registerCallback(boost::bind(&callback4, _1, _2, _3, _4));
      ros::spin();
      break;
    }
    case 5:{
      message_filters::Synchronizer<MySyncPolicy5> sync5(MySyncPolicy5(10), *subs[0], *subs[1], *subs[2], *subs[3], *subs[4]);
      sync5.registerCallback(boost::bind(&callback5, _1, _2, _3, _4, _5));
      ros::spin();
      break;
    }
    default:
      ROS_ERROR("Too many topics provided. Current limitation is of 5 kinects");
      return 1;
      break;
  }
  
  return 0;
}

void callback(const PCMsg::ConstPtr& first_msg_pc2){
  cout <<"in"<<endl;
  pcs_msg_pc2.clear();
  pcs_msg_pc2.push_back(first_msg_pc2);
  merge_pcs(pcs_msg_pc2);
}

void callback2(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2){
  cout <<"in"<<endl;
  pcs_msg_pc2.clear();
  pcs_msg_pc2.push_back(first_msg_pc2);
  pcs_msg_pc2.push_back(second_msg_pc2);
  merge_pcs(pcs_msg_pc2);
}

void callback3(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2 ){
  cout <<"in"<<endl;
  pcs_msg_pc2.clear();
  pcs_msg_pc2.push_back(first_msg_pc2);
  pcs_msg_pc2.push_back(second_msg_pc2);
  pcs_msg_pc2.push_back(third_msg_pc2);
  merge_pcs(pcs_msg_pc2);
}
void callback4(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2 ){
  cout <<"in"<<endl;
  pcs_msg_pc2.clear();
  pcs_msg_pc2.push_back(first_msg_pc2);
  pcs_msg_pc2.push_back(second_msg_pc2);
  pcs_msg_pc2.push_back(third_msg_pc2);
  pcs_msg_pc2.push_back(fourth_msg_pc2);
  merge_pcs(pcs_msg_pc2);
}
void callback5(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2 ){
  cout <<"in"<<endl;
  pcs_msg_pc2.clear();
  pcs_msg_pc2.push_back(first_msg_pc2);
  pcs_msg_pc2.push_back(second_msg_pc2);
  pcs_msg_pc2.push_back(third_msg_pc2);
  pcs_msg_pc2.push_back(fourth_msg_pc2);
  pcs_msg_pc2.push_back(fifth_msg_pc2);
  merge_pcs(pcs_msg_pc2);
}

void merge_pcs(vector<PCMsg::ConstPtr> pcs_msg_pc2){
  
  
}
