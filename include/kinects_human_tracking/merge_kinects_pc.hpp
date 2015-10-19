//| This file is a part of the sferes2 framework.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jimmy Da Silva, jimmy.dasilva@isir.upmc.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software. You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

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
   Subscribe to 1 to 9 pointclouds, transform into 
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
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg> MySyncPolicy6;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg> MySyncPolicy7;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg> MySyncPolicy8;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg, PCMsg> MySyncPolicy9;

struct pclTransform{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};

void callback9(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2, const PCMsg::ConstPtr& seventh_msg_pc2, const PCMsg::ConstPtr& eighth_msg_pc2, const PCMsg::ConstPtr& ninth_msg_pc2 );
void callback8(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2, const PCMsg::ConstPtr& seventh_msg_pc2, const PCMsg::ConstPtr& eighth_msg_pc2 );
void callback7(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2, const PCMsg::ConstPtr& seventh_msg_pc2 );
void callback6(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2 );
void callback5(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2 );
void callback4(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2);
void callback3(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2 );
void callback2(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2);
void callback(const PCMsg::ConstPtr& first_msg_pc2);
void merge_pcs(vector<PCMsg::ConstPtr> pcs_msg_pc2);
void get_transform(string pc_frame, pclTransform &pc_transform);

vector<pclTransform> transforms_;
bool first_frame_;
string out_frame_, out_topic_name_;
vector<string> in_topic_names_;
ros::Publisher pc_pub_;
vector<PointCloudSM::Ptr> pcs_;
vector<PCMsg::ConstPtr> pcs_msg_pc2_;
PointCloudSM::Ptr merged_pc_;