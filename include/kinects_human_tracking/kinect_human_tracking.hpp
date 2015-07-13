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
#include <tf/transform_listener.h>
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

/** \fn void callback(const pcl::pointCloud kinects_pc, const pcl::pointCloud robot_pc)
 *  \brief Take both human & robot pointClouds and returns min distance to robot and human position
 *  \param kinects_pc The pointCloud containing the humans
 *  \param robot_pc The robot's pointCloud
 */
void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg);

// Global variables
PointCloudSM::Ptr pcl_pc1_, clustered_cloud_;
pcl::PointCloud<pcl::PointXYZ>::Ptr robot_pc_;
ros::Publisher human_pc_pub_, pc_clustered_pub_, cloud_mini_pt_pub_, dist_pt_pub_, human_pose_pub_, human_pose_obs_pub_;
tf::TransformListener *tf_listener_;
double voxel_size_;
int min_cluster_size_;
Eigen::Vector2f last_human_pos_;
KalmanFilter kalman_;


// Templated functions declaration & definition

/** \fn void pc_downsampling(pcl::PointCloud<PointT>::Ptr pc_in, pcl::PointCloud<PointT>::Ptr pc_out, double& voxel_size){
 *  \brief Uses voxels to return downsampled version of the pointCloud 
 *  \param pc_in pointCloud to downsample
 *  \param pc_out poinCloud downsampled
 */
template<typename PointT> 
void pc_downsampling(boost::shared_ptr<pcl::PointCloud<PointT> >& pc_in, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out,double& voxel_size){
  
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud(pc_in);
  vox_grid.setLeafSize(voxel_size,voxel_size,voxel_size);
  vox_grid.filter(*pc_out);
  
}

/** \fn void pc_clipping(pcl::PointCloud<PointT>::Ptr pc_in, std::vector<ClippingRule> clipping_rules, pcl::PointCloud<PointT>::Ptr pc_clipped)
 *  \brief Removes parts of the cloud according to specified regions
 *  \param pc_in pointCloud to modify
 *  \param clipping_rules specifies regions we want to keep in the pointCloud
 *  \param pc_clipped poinCloud clipped
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
 *  \param pc_in pointCloud to cluster
 *  \param cluster_tolerance specifies the distance required between two points to be considered inside the same cluster
 *  \param clustered_pc clustered version of the pointCloud
 *  \return The indices corresponding to the clusters
 */
template<typename PointT>
vector<pcl::PointIndices> pc_clustering(boost::shared_ptr<pcl::PointCloud<PointT> >& pc_in, double cluster_tolerance, boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc){
  
  pcl::copyPointCloud(*pc_in, *clustered_pc);
  
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (clustered_pc);
  
  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (clustered_pc);
  ec.extract (cluster_indices);
  
  return cluster_indices;
  
}

/** \fn vector<ClusterStats> get_clusters_stats (pcl::PointCloud<PointT>::Ptr pc, vector<pcl::PointIndices> clusters_indices)
 *  \brief Computes statistics on the clusters
 *  \param pc pointCloud 
 *  \param clusters_indices The indices for each clusters
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
 *  \param pc pointCloud 
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
 *  \brief Computes the minimum distance between two poinClouds
 *  \param pc1 First poinCloud
 *  \param pc2 Second poinCloud
 *  \param min_dist Returned minimum distance value
 *  \param pc1_pt_min Returned closest point on the first poinCloud 
 *  \param pc2_pt_min Returned closest point on the second pointCloud
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

/** \fn void get_closest_cluster_to_robot(pcl::PointCloud<PointT>::Ptr human_clustered_pc, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<PointT2>::Ptr robot_pc, pcl::PointCloud<PointT>::Ptr pc_out, double& mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min)
 *  \brief Computes the closest cluster to the robot
 *  \param human_clustered_pc pointCloud containing the clusters
 *  \param cluster_indices Indices corresponding to the clusters in the poinCloud
 *  \param robot_pc Robot's pointCloud
 *  \param pc_out Returned pointCloud containing the closest cluster to the robot
 *  \param mini Returned minimum distance between the cluster and the robot
 *  \param pc1_pt_min Returned closest point on the first poinCloud 
 *  \param pc2_pt_min Returned closest point on the second pointCloud
 */
template<typename PointT, typename PointT2>
void get_closest_cluster_to_robot(boost::shared_ptr<pcl::PointCloud<PointT> >& human_clustered_pc, vector<pcl::PointIndices> cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT2> >& robot_pc, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double& mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min){
  
  // Check there is at least 1 cluster and that the robot cloud is published
  if ((cluster_indices.size() < 1) || (robot_pc->points.size() < 1))
    return;
  
  double min_dist;
  vector<double> min_dists;
  for(int i=0; i<cluster_indices.size();i++){
    pc_to_pc_min_dist(human_clustered_pc, robot_pc, min_dist, pc1_pt_min, pc2_pt_min);
    min_dists.push_back(min_dist);
  }
  
  mini = min_dists[0];
  int min_id = 0;
  for(int i=1; i<min_dists.size();i++){
    if(min_dists[i] < mini){
      mini = min_dists[i];
      min_id = i;
    } 
  }
  
  pcl::copyPointCloud(*human_clustered_pc,*pc_out);
  pc_out->points.clear();
  pcl::PointIndices idx = cluster_indices[min_id];
  for (std::vector<int>::const_iterator pit = idx.indices.begin(); pit != idx.indices.end(); ++pit)
      pc_out->points.push_back(human_clustered_pc->points[*pit]); 

  pc_out->width = pc_out->points.size();
  
}