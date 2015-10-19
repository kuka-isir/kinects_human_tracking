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

#ifndef KINECT_HUMAN_TRACKING
#define KINECT_HUMAN_TRACKING

//ros-includes
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kinects_human_tracking/kalmanFilter.hpp>
#include <kinects_human_tracking/pointCloudUtils.hpp>
/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then track him using a Kalman filter
 */

using namespace std;

// typedefs and tructs
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg> MySyncPolicy;

// Global variables
PointCloudSM::Ptr kinects_pc_, human_cloud_;
pcl::PointCloud<pcl::PointXYZI>::Ptr robot_pc_;
ros::Publisher human_pc_pub_, pc_clustered_pub_, cloud_mini_pt_pub_, dist_pt_pub_, human_state_pub_, mins_pub_;
double last_min_dist_, voxel_size_, kinect_noise_, process_noise_, minimum_height_, max_tracking_jump_;
std::vector<double> last_min_dists_;
geometry_msgs::PointStamped last_human_pt_, last_robot_pt_;
int min_cluster_size_;
Eigen::Vector2f last_human_pos_;
KalmanFilter kalman_;
ros::Time last_observ_time_;
vector<ClippingRule> clipping_rules_;
bool several_mins_;


// Functions declaration

/** \fn void callback(const pcl::pointCloud kinects_pc, const pcl::pointCloud robot_pc)
 *  \brief Take both human & robot pointClouds and returns min distance to robot and human position
 *  \param[in] kinects_pc The pointCloud containing the humans
 *  \param[in] robot_pc The robot's pointCloud
 */
void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg);

/** \fn void visualize_state (Eigen::Matrix<float, 6, 1> state, ClusterStats stats, ros::Publisher state_pub)
 *  \brief Take both human & robot pointClouds and returns min distance to robot and human position
 *  \param[in] state vector containing the estimated state of the human
 *  \param[in] stats stats on the human cluster
 *  \param[in] state_pub publisher to publish the markerArray cylinders
 */
void visualize_state (Eigen::Matrix<float, 6, 1> state, ClusterStats stats, ros::Publisher state_pub);

/** \fn void get_closest_cluster_to_robot(pcl::PointCloud<PointT>::Ptr clustered_pc, vector<pcl::PointIndices> cluster_indices, vector<pcl::PointCloud<PointT2>::Ptr> robot_pcs, pcl::PointCloud<PointT>::Ptr pc_out, vector<double> minis, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min)
 *  \brief Computes the closest cluster to a pointCloud and returns the distance between the cluster and each robot's link
 *  \param[in] clustered_pc pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the poinCloud
 *  \param[in] robot_pcs Vector containing the clouds corresponding to the different robot's link
 *  \param[out] pc_out Returned pointCloud containing the closest cluster to the robot
 *  \param[out] minis Returned minimum distances between the cluster and the robot links
 *  \param[out] pc1_pt_min Returned closest point on the first poinCloud 
 *  \param[out] pc2_pt_min Returned closest point on the second pointCloud
 */
template<typename PointT, typename PointT2>
void get_closest_cluster_to_robot(boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc, vector<pcl::PointIndices> cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT2> >& robot_pc, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, vector<double> minis, vector<int> min_human_id_vect, vector<int> min_robot_id_vect){
  
  const int nb_cluster = cluster_indices.size(), nb_links = 8;
  
  // Declare vector and matrices to save data 
  Eigen::MatrixXd mins(nb_cluster, nb_links);
  mins.fill(100000000.0);
  Eigen::MatrixXi min_robot_id(nb_cluster, nb_links);
  min_robot_id.fill(0);
  Eigen::MatrixXi min_cluster_id(nb_cluster, nb_links);
  min_cluster_id.fill(0);
  
  // Init marker
  visualization_msgs::Marker mini_lines_marker;
  geometry_msgs::Point pt;
  mini_lines_marker.header.frame_id = robot_pc->header.frame_id;
  mini_lines_marker.action = visualization_msgs::Marker::ADD;
  mini_lines_marker.id = 100;
  mini_lines_marker.scale.x = 0.01; 
  mini_lines_marker.lifetime = ros::Duration(0.0);
  mini_lines_marker.type = visualization_msgs::Marker::LINE_LIST;
  mini_lines_marker.color.r = 1.0;
  mini_lines_marker.color.a = 1.0;
  
  
  // Extract the pointCloud for each link
  vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > robot_pcs;
  robot_pcs.resize(nb_links);
  for(int i=0;i<nb_links;i++){
    robot_pcs[i] = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >(new pcl::PointCloud<pcl::PointXYZI>);
    robot_pcs[i]->header = robot_pc->header;
  }
  for (int i=0; i<robot_pc->points.size(); i++)
    robot_pcs[robot_pc->points[i].intensity]->points.push_back(robot_pc->points[i]);
  
  // Extract all clusters
  vector< typename pcl::PointCloud<PointT>::Ptr > pc_vect;
  std::vector<int> vect_i;
  for(int i=0;i<nb_cluster;i++){
    typename pcl::PointCloud<PointT>::Ptr temp_pc(new pcl::PointCloud<PointT>);
    vect_i.clear();
    vect_i.push_back(i);
    pc_extract_clusters(clustered_pc, cluster_indices, vect_i, temp_pc);
    pc_vect.push_back(temp_pc);
  }
  
  // for all clusters
  for(int c=0;c<nb_cluster;c++){
    // for all robot links
    for(int l=0;l<nb_links;l++){
      float min_dist = 100000000; 
      float dist = 0;
      // compute min_dist and save min_ids
      for (size_t i = 0; i < pc_vect[c]->points.size(); ++i){
	for (size_t j = 0; j < robot_pcs[l]->points.size(); ++j){  
	  pcl::Vector4fMap pt1 = pc_vect[c]->points[i].getVector4fMap();
	  pcl::Vector4fMap pt2 = robot_pcs[l]->points[j].getVector4fMap();
	  dist = (pt2 - pt1).norm();
	  if (dist < min_dist){
	    min_robot_id(c,l) = j;
	    min_cluster_id(c,l) = i;
	    mins(c,l) = dist;
	    min_dist = dist;
	  }
	}
      }
    }
  }
  
  // Look for the minimum in matrix
  std::ptrdiff_t idx, jdx;
  double minOfMins = mins.minCoeff(&idx,&jdx);

  minis.clear();
  for(int l=0;l<nb_links;l++){
    Eigen::MatrixXd col = mins.col(l);
    
    std::ptrdiff_t idx, jdx;
    minis.push_back(col.minCoeff(&idx,&jdx));
    min_robot_id_vect.push_back(min_robot_id(idx,l));
    min_human_id_vect.push_back(min_cluster_id(idx,l));
    
    pt.x = pc_vect[idx]->points[min_cluster_id(idx,l)].x;
    pt.y = pc_vect[idx]->points[min_cluster_id(idx,l)].y;
    pt.z = pc_vect[idx]->points[min_cluster_id(idx,l)].z;
    mini_lines_marker.points.push_back(pt);
    pt.x = robot_pcs[l]->points[min_robot_id(idx,l)].x;
    pt.y = robot_pcs[l]->points[min_robot_id(idx,l)].y;
    pt.z = robot_pcs[l]->points[min_robot_id(idx,l)].z;
    mini_lines_marker.points.push_back(pt);
  }
  
  *pc_out = *pc_vect[idx];
  
  mins_pub_.publish(mini_lines_marker);
  
}

#endif
