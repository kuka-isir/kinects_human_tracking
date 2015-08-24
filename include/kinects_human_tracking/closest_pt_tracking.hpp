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
ros::Publisher cluster_pc_pub_, pc_clustered_pub_, cloud_mini_pt_pub_, dist_pt_pub_, cluster_state_pub_, mins_pub_, min_pub_,vel_pub_;
double last_min_dist_, voxel_size_, kinect_noise_, process_noise_, minimum_height_, max_tracking_jump_, clustering_tolerance_;
geometry_msgs::PointStamped last_cluster_pt_;
int min_cluster_size_;
Eigen::Vector3f last_pos_;
KalmanFilter kalman_;
ros::WallTime last_observ_time_;
vector<ClippingRule> clipping_rules_;
tf::TransformListener* tf_listener_;

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