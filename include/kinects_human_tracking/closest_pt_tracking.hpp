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

//eigen include
#include <Eigen/Eigen>
#include <eigen3/Eigen/Eigen>

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
#include <boost/graph/graph_concepts.hpp>

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
#include <kinects_human_tracking/kalmanFilter3Pos3Vel.hpp>
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
PointCloudSM::Ptr kinects_pc_, cluster_cloud_;
ros::Publisher cluster_pc_pub_, pc_clustered_pub_, cloud_mini_pt_pub_, dist_pt_pub_, cluster_state_pub_, mins_pub_;
double last_min_dist_, voxel_size_, kinect_noise_, process_noise_, minimum_height_, max_tracking_jump_;
std::vector<double> last_min_dists_;
geometry_msgs::PointStamped last_cluster_pt_;
int min_cluster_size_;
Eigen::Vector3f last_pos_;
KalmanFilter kalman_;
ros::Time last_observ_time_;
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
  
  pc_out->header.frame_id = pc_in->header.frame_id;
    
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

/** \fn void pc_to_frame_min_dist(pcl::PointCloud<PointT>::Ptr pc, std::string frame_id, double& min_dist, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the minimum distance between a pointCloud and a frame
 *  \param[in] pc pointCloud
 *  \param[in] frame_id frame
 *  \param[out] min_dist Returned minimum distance value
 *  \param[out] pc1_pt_min Returned closest point on the pointCloud 
 */
template<typename PointT> 
void pc_to_frame_min_dist(boost::shared_ptr<pcl::PointCloud<PointT> >& pc, std::string frame_id, double& min_dist, geometry_msgs::PointStamped& pc_pt_min){
  
  tf::StampedTransform transform;
  tf_listener_->waitForTransform("world", frame_id, ros::Time(0.0), ros::Duration(1.0));
  tf_listener_->lookupTransform("world", frame_id, ros::Time(0.0), transform);
  
  Eigen::Vector4f pt2(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ(), 1.0);
  
  min_dist = 100000000;
  float dist = 0; 
  int min_idx = 0;
  for (size_t i = 0; i < pc->points.size (); ++i){
    pcl::Vector4fMap pt1 = pc->points[i].getVector4fMap();
    dist = (pt2 - pt1).norm ();
    if (dist < min_dist){
      min_idx = i;
      min_dist = dist;
    }
  }
  
  pc_pt_min.header.frame_id = pc->header.frame_id;
  pc_pt_min.point.x = pc->points[min_idx].x;
  pc_pt_min.point.y = pc->points[min_idx].y;
  pc_pt_min.point.z = pc->points[min_idx].z; 
    
}

/** \fn void get_closest_cluster_to_robot(pcl::PointCloud<PointT>::Ptr clustered_pc, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<PointT2>::Ptr robot_pc, pcl::PointCloud<PointT>::Ptr pc_out, double mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min)
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
void get_closest_cluster_to_robot(boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc, vector<pcl::PointIndices> cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT2> >& robot_pc, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double mini, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min){
  
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

/** \fn void get_closest_cluster_to_robot(pcl::PointCloud<PointT>::Ptr clustered_pc, vector<pcl::PointIndices> cluster_indices, vector<pcl::PointCloud<PointT2>::Ptr> robot_pcs, pcl::PointCloud<PointT>::Ptr pc_out, vector<double> minis, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min)
 *  \brief Computes the closest cluster to the robot and returns the distance between the cluster and each robot's link
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

/** \fn void get_closest_pt_to_frame(pcl::PointCloud<PointT>::Ptr clustered_pc, vector<pcl::PointIndices> cluster_indices, std::string frame_id, pcl::PointCloud<PointT>::Ptr pc_out, double min_dist, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the closest cluster to the robot and returns the distance between the cluster and each robot's link
 *  \param[in] clustered_pc pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the poinCloud
 *  \param[in] frame_id Frame form which to compute the distances
 *  \param[out] pc_out Returned pointCloud containing the closest cluster to the robot
 *  \param[out] min_dist Returned minimum distance between the clusters and the frame
 *  \param[out] pc_pt_min Returned closest point on the first poinCloud 
 */
template<typename PointT>
void get_closest_pt_to_frame(boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc, vector<pcl::PointIndices> cluster_indices, std::string frame_id, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double& min_dist, geometry_msgs::PointStamped& pc_pt_min){
   
  // Check there is at least 1 cluster
  if ((cluster_indices.size() < 1))
    return;
  
  vector<double> min_dists;
  std::vector<int> cluster_id;
  typename pcl::PointCloud<PointT>::Ptr temp_pc(new pcl::PointCloud<PointT>);
  typename std::vector< pcl::PointCloud<PointT> > temp_pcs;
  std::vector< geometry_msgs::PointStamped> pc_pt_mins;
  for(int i=0; i<cluster_indices.size();i++){
    cluster_id.clear();
    cluster_id.push_back(i);
    
    pc_extract_clusters(clustered_pc, cluster_indices, cluster_id, temp_pc);
    pc_to_frame_min_dist(temp_pc, frame_id, min_dist, pc_pt_min);
    
    min_dists.push_back(min_dist);
    temp_pcs.push_back(*temp_pc);
    pc_pt_mins.push_back(pc_pt_min);
  }
  
  min_dist = min_dists[0];
  int min_id = 0;
  for(int i=0; i<min_dists.size();i++){
    if(min_dists[i] < min_dist){
      min_dist = min_dists[i];
      min_id = i;
    } 
  }
  
  *pc_out = temp_pcs[min_id];
  pc_pt_min = pc_pt_mins[min_id];
  
}
#endif