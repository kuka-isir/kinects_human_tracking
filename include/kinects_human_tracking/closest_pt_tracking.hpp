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

#ifndef CLOSEST_PT_TRACKING
#define CLOSEST_PT_TRACKING

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <kinects_human_tracking/kalmanFilter3Pos3Vel.hpp>
#include <kinects_human_tracking/pointCloudUtils.hpp>

// defining pi
#ifndef M_PI
#define M_PI 3.141592653589793
#endif
/**
   Subscribe to a pointCloud and track the closest
   point to the robot's end-effector
 */

using namespace std;

// typedefs and tructs
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;

// Global variables
PointCloudSM::Ptr kinects_pc_, cluster_cloud_;
ros::Publisher cluster_pc_pub_, pc_clustered_pub_, cloud_mini_pt_pub_, dist_pt_pub_, cluster_state_pub_, mins_pub_, min_pub_,vel_pub_, dist_vect_pub_;
double last_min_dist_, voxel_size_, kinect_noise_, kinect_noise_z_, process_noise_, minimum_height_, max_tracking_jump_, clustering_tolerance_;
bool downsampling_;
geometry_msgs::PointStamped last_cluster_pt_;
int min_cluster_size_;
Eigen::Vector3f last_pos_;
KalmanFilter kalman_;
ros::Time last_observ_time_;
vector<ClippingRule> clipping_rules_;
tf::TransformListener* tf_listener_;
string enf_eff_frame_;

// Functions declaration

/** \fn void callback(const pcl::pointCloud kinects_pc)
 *  \brief Take both kinect pointCloud and returns min distance to robot and closest object position
 *  \param[in] kinects_pc The pointCloud containing the humans
 */
void callback(const PCMsg::ConstPtr& kinects_pc);

/** \fn void visualize_state (Eigen::Matrix<float, 9, 1> state, ros::Publisher state_pub)
 *  \brief Visualize the tracking position and velocity
 *  \param[in] state vector containing the estimated state of the human
 *  \param[in] state_pub publisher to publish the markerArray
 */
void visualize_state (Eigen::Matrix<float, 9, 1> state, ros::Publisher state_pub);

#endif
