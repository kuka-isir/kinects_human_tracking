#ifndef KINECT_HUMAN_TRACKING
#define KINECT_HUMAN_TRACKING

//pcl includes
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>

//boost includes
#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/timer.hpp>

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
/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then track him using a Kalman filter
 */

using namespace std;

// typedefs and tructs
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg> MySyncPolicy;
typedef Eigen::Vector3f ClusterPoint;

/** \struct ClusterStats
 *  Used to save min, max, mean, median and variance stats of a cluster
 */
struct ClusterStats{
  ClusterPoint mean;
  ClusterPoint var; 
  ClusterPoint min; 
  ClusterPoint max; 
  ClusterPoint median;
};
typedef struct ClusterStats ClusterStats;

/** \struct ClippingRule
 *  Used to specify regions or the world to keep in the pointclouds
 */
struct ClippingRule{
  string axis; 	// x, y or z
  string op;	// GT or LT
  double val;
};
typedef struct ClippingRule ClippingRule;

// Global variables
PointCloudSM::Ptr kinects_pc_, human_cloud_;
pcl::PointCloud<pcl::PointXYZI>::Ptr robot_pc_;
ros::Publisher human_pc_pub_, pc_clustered_pub_, cloud_mini_pt_pub_, dist_pt_pub_, human_state_pub_;
double last_min_dist_, voxel_size_, kinect_noise_, process_noise_, minimum_height_, max_tracking_jump_;
geometry_msgs::PointStamped last_human_pt_, last_robot_pt_;
int min_cluster_size_;
Eigen::Vector2f last_human_pos_;
KalmanFilter kalman_;
ros::Time last_observ_time_;
vector<ClippingRule> clipping_rules_;


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


// Templated functions declaration & definition

/** \fn void pc_downsampling(pcl::PointCloud<PointT>::Ptr pc_in, double& voxel_size, pcl::PointCloud<PointT>::Ptr pc_out){
 *  \brief Uses voxels to return downsampled version of the pointCloud 
 *  \param[in] pc_in pointCloud to downsample
 *  \param[in] voxel_size distance wanted between each points
 *  \param[out] pc_out poinCloud downsampled
 */
template<typename PointT> 
void pc_downsampling(boost::shared_ptr<pcl::PointCloud<PointT> >& pc_in, double& voxel_size, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out){
  
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud(pc_in);
  vox_grid.setLeafSize(voxel_size_,voxel_size_,voxel_size_);
  vox_grid.filter(*pc_out);
  
}

/** \fn void pc_clipping(pcl::PointCloud<PointT>::Ptr pc_in, std::vector<ClippingRule> clipping_rules, pcl::PointCloud<PointT>::Ptr pc_clipped)
 *  \brief Removes parts of the cloud according to specified regions
 *  \param[in] pc_in pointCloud to modify
 *  \param[in] clipping_rules specifies regions we want to keep in the pointCloud
 *  \param[out] pc_clipped poinCloud clipped
 */
template<typename PointT> 
void pc_clipping(boost::shared_ptr<pcl::PointCloud<PointT> >& pc_in, std::vector<ClippingRule> clipping_rules, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_clipped){
  
  typename pcl::ConditionAnd<PointT>::Ptr height_cond (new pcl::ConditionAnd<PointT> ());
  for(size_t i=0; i<clipping_rules.size(); i++){
  
    string axis = clipping_rules[i].axis;
    string op = clipping_rules[i].op;
    double val = clipping_rules[i].val;
    
    if ((axis == "x") || (axis == "y") || (axis == "z")){ 
      if ((op == "LT") || (op=="GT")){
	if (op == "LT")
	  height_cond->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> (axis, pcl::ComparisonOps::LT, val)));
	else
	  height_cond->addComparison (typename pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> (axis, pcl::ComparisonOps::GT, val)));
      }
      else{
      ROS_ERROR("Clipping rules not valid!!!\n Please use \'GT\' or \'LT\' for the op attribute");
      return;
      }
    }
    else{
      ROS_ERROR("Clipping rules not valid!!!\n Please use \'x\' \'y\' \'z\' for the axis");
      return;
    }
  }
  
  pcl::ConditionalRemoval<PointT> condrem (height_cond);
  condrem.setInputCloud (pc_in);
  condrem.setKeepOrganized(true);
  condrem.filter (*pc_clipped); 
  
}

/** \fn vector<pcl::PointIndices> pc_clustering(pcl::PointCloud<PointT>::Ptr pc_in, double cluster_tolerance, pcl::PointCloud<PointT>::Ptr pc_out)
 *  \brief Finds out the clusters inside the given poinCloud 
 *  \param[in] pc_in pointCloud to cluster
 *  \param[in] cluster_tolerance specifies the distance required between two points to be considered inside the same cluster
 *  \param[out] clustered_pc clustered version of the pointCloud
 *  \return The indices corresponding to the clusters
 */
template<typename PointT>
vector<pcl::PointIndices> pc_clustering(boost::shared_ptr<pcl::PointCloud<PointT> >& pc_in, double cluster_tolerance, boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc){
  
  pcl::copyPointCloud(*pc_in, *clustered_pc);
  
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (pc_in);
  
  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pc_in);
  ec.extract (cluster_indices);
  
  return cluster_indices;
  
}

/** \fn void pc_extract_clusters(pcl::PointCloud<PointT>::Ptr pc_in, vector<pcl::PointIndices> cluster_indices, vector<int> cluster_ids, pcl::PointCloud<PointT2>::Ptr pc_out)
 *  \brief Computes the closest cluster to the robot
 *  \param[in] pc_in Full pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the poinCloud
 *  \param[in] cluster_ids Ids of the clusters to extract from the full pointCloud
 *  \param[out] pc_out Returned pointCloud containing the desired clusters
 */
template<typename PointT, typename PointT2>
void pc_extract_clusters(boost::shared_ptr<pcl::PointCloud<PointT> >& pc_in, vector<pcl::PointIndices> cluster_indices, vector<int> cluster_ids, boost::shared_ptr<pcl::PointCloud<PointT2> >& pc_out){
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  for(int i=0; i<cluster_ids.size(); i++){
    for(int j=0; j<cluster_indices[cluster_ids[i]].indices.size(); j++)
      inliers->indices.push_back(cluster_indices[cluster_ids[i]].indices[j]);
  }
  
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (pc_in);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*pc_out);
    
}

/** \fn vector<ClusterStats> get_clusters_stats (pcl::PointCloud<PointT>::Ptr pc, vector<pcl::PointIndices> clusters_indices)
 *  \brief Computes statistics on the clusters
 *  \param[in] pc pointCloud 
 *  \param[out] clusters_indices The indices for each clusters
 *  \return Returns a vector containing the stats of each clusters
 */
template<typename PointT> 
vector<ClusterStats> get_clusters_stats (boost::shared_ptr<pcl::PointCloud<PointT> >& pc, vector<pcl::PointIndices> clusters_indices){
  
  std::vector<ClusterStats> stats;
  
  for(int i = 0; i<clusters_indices.size(); i++){
    boost::accumulators::accumulator_set< float, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance, boost::accumulators::tag::min, boost::accumulators::tag::max, boost::accumulators::tag::median> > x_acc, y_acc, z_acc; 
    for(vector<int>::const_iterator pint = clusters_indices[i].indices.begin(); pint!=clusters_indices[i].indices.end(); ++pint){
      PointT p = pc->points[*pint];
      x_acc(p.x);
      y_acc(p.y);
      z_acc(p.z);
    }
    ClusterStats cluster_stats;
    cluster_stats.mean = ClusterPoint(boost::accumulators::mean(x_acc), boost::accumulators::mean(y_acc), boost::accumulators::mean(z_acc));
    cluster_stats.var = ClusterPoint(boost::accumulators::variance(x_acc), boost::accumulators::variance(y_acc), boost::accumulators::variance(z_acc));
    cluster_stats.min = ClusterPoint(boost::accumulators::min(x_acc), boost::accumulators::min(y_acc), boost::accumulators::min(z_acc));    
    cluster_stats.max = ClusterPoint(boost::accumulators::max(x_acc), boost::accumulators::max(y_acc), boost::accumulators::max(z_acc));    
    cluster_stats.median = ClusterPoint(boost::accumulators::median(x_acc), boost::accumulators::median(y_acc), boost::accumulators::median(z_acc));    
    
    stats.push_back(cluster_stats);
  } 
  return stats;
}

/** \fn ClusterStats get_cluster_stats (pcl::PointCloud<PointT>::Ptr pc)
 *  \brief Removes parts of the cloud according to specified regions
 *  \param[in] pc pointCloud 
 *  \return Returns the stats for the given cluster
 */
template<typename PointT> 
ClusterStats get_cluster_stats (boost::shared_ptr<pcl::PointCloud<PointT> >& pc){
  
  boost::accumulators::accumulator_set< float, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance, boost::accumulators::tag::min, boost::accumulators::tag::max, boost::accumulators::tag::median> > x_acc, y_acc, z_acc; 
  for(int i=0; i< pc->points.size();i++ ){
    PointT p = pc->points[i];
    x_acc(p.x);
    y_acc(p.y);
    z_acc(p.z);
  }
  ClusterStats cluster_stats;
  cluster_stats.mean = ClusterPoint(boost::accumulators::mean(x_acc), boost::accumulators::mean(y_acc), boost::accumulators::mean(z_acc));
  cluster_stats.var = ClusterPoint(boost::accumulators::variance(x_acc), boost::accumulators::variance(y_acc), boost::accumulators::variance(z_acc));
  cluster_stats.min = ClusterPoint(boost::accumulators::min(x_acc), boost::accumulators::min(y_acc), boost::accumulators::min(z_acc));    
  cluster_stats.max = ClusterPoint(boost::accumulators::max(x_acc), boost::accumulators::max(y_acc), boost::accumulators::max(z_acc));    
  cluster_stats.median = ClusterPoint(boost::accumulators::median(x_acc), boost::accumulators::median(y_acc), boost::accumulators::median(z_acc));    
    
  return cluster_stats;
}

/** \fn void pc_to_pc_min_dist(pcl::PointCloud<PointT>::Ptr pc1, pcl::PointCloud<PointT2>::Ptr pc2, double& min_dist, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min)
 *  \brief Computes the minimum distance between two pointClouds
 *  \param[in] pc1 First pointCloud
 *  \param[in] pc2 Second pointCloud
 *  \param[out] min_dist Returned minimum distance value
 *  \param[out] pc1_pt_min Returned closest point on the first pointCloud 
 *  \param[out] pc2_pt_min Returned closest point on the second pointCloud
 */
template<typename PointT, typename PointT2> 
void pc_to_pc_min_dist(boost::shared_ptr<pcl::PointCloud<PointT> >& pc1, boost::shared_ptr<pcl::PointCloud<PointT2> >& pc2, double& min_dist, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min){
  
  min_dist = 100000000;
  float dist = 0; 
  int min_idx = 0;
  int min_jdx = 0;
  for (size_t i = 0; i < pc1->points.size (); ++i){
    for (size_t j = 0; j < pc2->points.size (); ++j){  
      pcl::Vector4fMap pt1 = pc1->points[i].getVector4fMap();
      pcl::Vector4fMap pt2 = pc2->points[j].getVector4fMap();
      dist = (pt2 - pt1).norm ();
      if (dist < min_dist){
	min_idx = i;
	min_jdx = j;
	min_dist = dist;
      }
    }
  }
  
  pc1_pt_min.header.frame_id = pc1->header.frame_id;
  pc1_pt_min.point.x = pc1->points[min_idx].x;
  pc1_pt_min.point.y = pc1->points[min_idx].y;
  pc1_pt_min.point.z = pc1->points[min_idx].z; 
  
  pc2_pt_min.header.frame_id = pc2->header.frame_id;
  pc2_pt_min.point.x = pc2->points[min_jdx].x;
  pc2_pt_min.point.y = pc2->points[min_jdx].y;
  pc2_pt_min.point.z = pc2->points[min_jdx].z;
  
}

/** \fn void get_closest_cluster_to_robot(pcl::PointCloud<PointT>::Ptr clustered_pc, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<PointT2>::Ptr robot_pc, pcl::PointCloud<PointT>::Ptr pc_out, double& mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min)
 *  \brief Computes the closest cluster to the robot
 *  \param[in] clustered_pc pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the poinCloud
 *  \param[in] robot_pc Robot's pointCloud
 *  \param[out] pc_out Returned pointCloud containing the closest cluster to the robot
 *  \param[out] mini Returned minimum distance between the cluster and the robot
 *  \param[out] pc1_pt_min Returned closest point on the first poinCloud 
 *  \param[out] pc2_pt_min Returned closest point on the second pointCloud
 */
template<typename PointT, typename PointT2>
void get_closest_cluster_to_robot(boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc, vector<pcl::PointIndices> cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT2> >& robot_pc, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double& mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min){
  
  // Check there is at least 1 cluster and that the robot cloud is published
  if ((cluster_indices.size() < 1) || (robot_pc->points.size() < 1))
    return;
  
  double min_dist;
  vector<double> min_dists;
  std::vector<int> cluster_id;
  typename pcl::PointCloud<PointT>::Ptr temp_pc(new pcl::PointCloud<PointT>);
  typename std::vector< pcl::PointCloud<PointT> > temp_pcs;
  std::vector< geometry_msgs::PointStamped> pc1_pt_mins, pc2_pt_mins;
  for(int i=0; i<cluster_indices.size();i++){
    cluster_id.clear();
    cluster_id.push_back(i);
    
    pc_extract_clusters(clustered_pc, cluster_indices, cluster_id, temp_pc);
    pc_to_pc_min_dist(temp_pc, robot_pc, min_dist, pc1_pt_min, pc2_pt_min);
    
    min_dists.push_back(min_dist);
    temp_pcs.push_back(*temp_pc);
    pc1_pt_mins.push_back(pc1_pt_min);
    pc2_pt_mins.push_back(pc2_pt_min);
  }
  
  mini = min_dists[0];
  int min_id = 0;
  for(int i=0; i<min_dists.size();i++){
    if(min_dists[i] < mini){
      mini = min_dists[i];
      min_id = i;
    } 
  }
  
  *pc_out = temp_pcs[min_id];
  pc1_pt_min = pc1_pt_mins[min_id];
  pc2_pt_min = pc2_pt_mins[min_id];
  
}
#endif