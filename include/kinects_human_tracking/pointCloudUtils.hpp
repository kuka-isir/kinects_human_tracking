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

/**
 * Library for pcl::pointCloud manipulation
 *
 * \author Jimmy Da Silva, ISIR
 * 
 */

#ifndef POINTCLOUD_UTILS
#define POINTCLOUD_UTILS

// pcl includes
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

// eigen include
#include <Eigen/Eigen>
#include <eigen3/Eigen/Eigen>

// boost includes
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

// ros-includes
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

typedef Eigen::Vector3f ClusterPoint;

/** \struct ClippingRule
 *  Used to specify regions or the world to keep in the pointclouds
 */
struct ClippingRule{
  std::string axis; 	// x, y or z
  std::string op;	// GT or LT
  double val;
};
typedef struct ClippingRule ClippingRule;

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

/**
 * Templated functions for pointCloud manipulation
 */

/** \fn void pc_downsampling(pcl::PointCloud<PointT>::Ptr pc_in, double voxel_size, pcl::PointCloud<PointT>::Ptr pc_out){
 *  \brief Downsample a pointcloud with a voxel grid
 *  \param[in] pc_in pointCloud in
 *  \param[in] voxel_size voxel size
 *  \param[out] pc_out poinCloud out
 */
template<typename PointT> 
void pc_downsampling(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, double voxel_size, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out){
  
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud(pc_in);
  vox_grid.setLeafSize(voxel_size,voxel_size,voxel_size);
  vox_grid.filter(*pc_out);
  
}

/** \fn void pc_clipping(pcl::PointCloud<PointT>::Ptr pc_in, std::vector<ClippingRule> clipping_rules, pcl::PointCloud<PointT>::Ptr pc_clipped)
 *  \brief Removes parts of the cloud according to specified regions
 *  \param[in] pc_in pointCloud in
 *  \param[in] clipping_rules specifies regions we want to keep in the pointCloud
 *  \param[out] pc_clipped poinCloud out
 */
template<typename PointT> 
void pc_clipping(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, std::vector<ClippingRule> clipping_rules, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_clipped){
  
  typename pcl::ConditionAnd<PointT>::Ptr height_cond (new pcl::ConditionAnd<PointT> ());
  for(size_t i=0; i<clipping_rules.size(); i++){
  
    std::string axis = clipping_rules[i].axis;
    std::string op = clipping_rules[i].op;
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

/** \fn vector<pcl::PointIndices> pc_clustering(pcl::PointCloud<PointT>::Ptr pc_in, int min_cluster_size, double cluster_tolerance, pcl::PointCloud<PointT>::Ptr pc_out)
 *  \brief Finds out the clusters inside the given pointCloud 
 *  \param[in] pc_in pointCloud in
 *  \param[in] min_cluster_size minimum number of points to consider a cluster
 *  \param[in] cluster_tolerance longest distance between two points to be considered in the same cluster
 *  \param[out] clustered_pc pointCloud out
 *  \return The indices corresponding to the clusters
 */
template<typename PointT>
std::vector<pcl::PointIndices> pc_clustering(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, int min_cluster_size, double cluster_tolerance, boost::shared_ptr<pcl::PointCloud<PointT> >& clustered_pc){
  
  pcl::copyPointCloud(*pc_in, *clustered_pc);
  
  std::vector<pcl::PointIndices> cluster_indices;
  if (pc_in->points.size()==0){
    return cluster_indices;
  }
  
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (pc_in);
  
  typename pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pc_in);
  ec.extract (cluster_indices);
  
  return cluster_indices;
}

/** \fn void pc_extract_clusters(pcl::PointCloud<PointT>::Ptr pc_in, std::vector<pcl::PointIndices> cluster_indices, std::vector<int> cluster_ids, pcl::PointCloud<PointT2>::Ptr pc_out)
 *  \brief Extracts the cluster from the pointCloud
 *  \param[in] pc_in pointCloud in
 *  \param[in] cluster_indices Indices corresponding to the clusters in the pointCloud
 *  \param[in] cluster_ids Ids of the clusters to extract from the full pointCloud
 *  \param[out] pc_out pointCloud out
 */
template<typename PointT, typename PointT2>
void pc_extract_clusters(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, std::vector<pcl::PointIndices> cluster_indices, std::vector<int> cluster_ids, boost::shared_ptr<pcl::PointCloud<PointT2> >& pc_out){
  
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

/** \fn std::vector<ClusterStats> get_clusters_stats (pcl::PointCloud<PointT>::Ptr pc, std::vector<pcl::PointIndices> clusters_indices)
 *  \brief Computes statistics on the clusters: min, max, mean, median, variance(var)
 *  \param[in] pc pointCloud in
 *  \param[in] clusters_indices The indices for each clusters
 *  \return A vector containing the stats of each clusters
 */
template<typename PointT> 
std::vector<ClusterStats> get_clusters_stats (boost::shared_ptr<pcl::PointCloud<PointT> > pc, std::vector<pcl::PointIndices> clusters_indices){
  
  std::vector<ClusterStats> stats;
  
  for(int i = 0; i<clusters_indices.size(); i++){
    boost::accumulators::accumulator_set< float, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance, boost::accumulators::tag::min, boost::accumulators::tag::max, boost::accumulators::tag::median> > x_acc, y_acc, z_acc; 
    for(std::vector<int>::const_iterator pint = clusters_indices[i].indices.begin(); pint!=clusters_indices[i].indices.end(); ++pint){
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

/** \fn ClusterStats get_pc_stats (pcl::PointCloud<PointT>::Ptr pc)
 *  \brief Computes statistics on the pointCloud: min, max, mean, median, variance(var)
 *  \param[in] pc pointCloud 
 *  \return Returns the stats for the given pointCloud
 */
template<typename PointT> 
ClusterStats get_pc_stats (boost::shared_ptr<pcl::PointCloud<PointT> > pc){
  
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

/** \fn void pc_to_frame_min_dist(pcl::PointCloud<PointT>::Ptr pc, tf::StampedTransform transform, double& min_dist, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the minimum distance between a pointCloud and a frame
 *  \param[in] pc pointCloud
 *  \param[in] transform transformation between the pointCloud frame and the target frame
 *  \param[out] min_dist Returned minimum distance value
 *  \param[out] pc1_pt_min Returned closest point in the pointCloud to target frame
 */
template<typename PointT> 
void pc_to_frame_min_dist(boost::shared_ptr<pcl::PointCloud<PointT> > pc, tf::StampedTransform transform, double& min_dist, geometry_msgs::PointStamped& pc_pt_min){
  
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

/** \fn void pc_to_frame_min_dist(pcl::PointCloud<PointT>::Ptr pc, tf::TransformListener* tf_listener, std::string frame_id, double& min_dist, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the minimum distance between a pointCloud and a frame
 *  \param[in] pc pointCloud
 *  \param[in] tf_listener tf::TransformListener*
 *  \param[in] frame_id Frame from which to compute the distance
 *  \param[out] min_dist Returned minimum distance value
 *  \param[out] pc1_pt_min Returned closest point in the pointCloud to target frame
 */
template<typename PointT> 
void pc_to_frame_min_dist(boost::shared_ptr<pcl::PointCloud<PointT> > pc, tf::TransformListener* tf_listener, std::string frame_id, double& min_dist, geometry_msgs::PointStamped& pc_pt_min){
  
  tf::StampedTransform transform;
  try{
    tf_listener->waitForTransform(pc->header.frame_id, frame_id, ros::Time(0.0), ros::Duration(1.0));
    tf_listener->lookupTransform(pc->header.frame_id, frame_id, ros::Time(0.0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  pc_to_frame_min_dist<PointT>(pc, transform, min_dist, pc_pt_min);
    
}

/** \fn void get_closest_cluster_to_frame(pcl::PointCloud<PointT>::Ptr clustered_pc, std::vector<pcl::PointIndices> cluster_indices, std::string frame_id, pcl::PointCloud<PointT>::Ptr pc_out, double min_dist, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the closest cluster to the frame
 *  \param[in] pc_in pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the pointCloud
 *  \param[in] transform transformation between the pointCloud frame and the target frame
 *  \param[out] pc_out Returned pointCloud containing the closest cluster to the robot
 *  \param[out] min_dist Returned minimum distance between the clusters and the frame
 *  \param[out] pc_pt_min Returned closest point on the first pointCloud 
 */
template<typename PointT>
void get_closest_cluster_to_frame(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, std::vector<pcl::PointIndices> cluster_indices, tf::StampedTransform transform, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double& min_dist, geometry_msgs::PointStamped& pc_pt_min){
   
  // Check there is at least 1 cluster
  if ((cluster_indices.size() < 1))
    return;
  
  std::vector<double> min_dists;
  std::vector<int> cluster_id;
  typename pcl::PointCloud<PointT>::Ptr temp_pc(new pcl::PointCloud<PointT>);
  typename std::vector< pcl::PointCloud<PointT> > temp_pcs;
  std::vector< geometry_msgs::PointStamped> pc_pt_mins;
  for(int i=0; i<cluster_indices.size();i++){
    cluster_id.clear();
    cluster_id.push_back(i);
    
    pc_extract_clusters(pc_in, cluster_indices, cluster_id, temp_pc);
    pc_to_frame_min_dist(temp_pc, transform, min_dist, pc_pt_min);
    
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

/** \fn void get_closest_cluster_to_frame(pcl::PointCloud<PointT>::Ptr clustered_pc, std::vector<pcl::PointIndices> cluster_indices, std::string frame_id, pcl::PointCloud<PointT>::Ptr pc_out, double min_dist, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the closest cluster to the frame
 *  \param[in] pc_in pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the pointCloud
 *  \param[in] tf::TransformListener*
 *  \param[in] frame_id Frame from which to compute the distance
 *  \param[out] pc_out Returned pointCloud containing the closest cluster to the robot
 *  \param[out] min_dist Returned minimum distance between the clusters and the frame
 *  \param[out] pc_pt_min Returned closest point on the first pointCloud 
 */
template<typename PointT>
void get_closest_cluster_to_frame(boost::shared_ptr<pcl::PointCloud<PointT> > pc_in, std::vector<pcl::PointIndices> cluster_indices, tf::TransformListener* tf_listener, std::string frame_id, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double& min_dist, geometry_msgs::PointStamped& pc_pt_min){
   
  // Check there is at least 1 cluster
  if ((cluster_indices.size() < 1))
    return;
  
  tf::StampedTransform transform;
  try{
    tf_listener->waitForTransform(pc_in->header.frame_id, frame_id, ros::Time(0.0), ros::Duration(1.0));
    tf_listener->lookupTransform(pc_in->header.frame_id, frame_id, ros::Time(0.0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  get_closest_cluster_to_frame(pc_in, cluster_indices, transform, pc_out, min_dist, pc_pt_min);
  
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
void pc_to_pc_min_dist(boost::shared_ptr<pcl::PointCloud<PointT> > pc1, boost::shared_ptr<pcl::PointCloud<PointT2> > pc2, double& min_dist, geometry_msgs::PointStamped& pc1_pt_min, geometry_msgs::PointStamped& pc2_pt_min){
  
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

/** \fn void get_closest_cluster_to_pc(pcl::PointCloud<PointT>::Ptr clustered_pc, vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<PointT2>::Ptr pc_in, pcl::PointCloud<PointT>::Ptr pc_out, double mini, geometry_msgs::PointStamped& cluster_pt_min, geometry_msgs::PointStamped& pc_pt_min)
 *  \brief Computes the closest cluster to a pointCloud
 *  \param[in] clustered_pc pointCloud containing the clusters
 *  \param[in] cluster_indices Indices corresponding to the clusters in the poinCloud
 *  \param[in] pc_in Robot's pointCloud
 *  \param[out] pc_out Returned pointCloud containing the closest cluster to the pointcloud
 *  \param[out] mini Returned minimum distance between the cluster and the poitCloud
 *  \param[out] cluster_pt_min Returned closest point in the cluster
 *  \param[out] pc_pt_min Returned closest point in the pointCloud
 */
template<typename PointT, typename PointT2>
void get_closest_cluster_to_pc(boost::shared_ptr<pcl::PointCloud<PointT> > clustered_pc, std::vector<pcl::PointIndices> cluster_indices, boost::shared_ptr<pcl::PointCloud<PointT2> > pc_in, boost::shared_ptr<pcl::PointCloud<PointT> >& pc_out, double mini, geometry_msgs::PointStamped& cluster_pt_min, geometry_msgs::PointStamped& pc_pt_min){
  
  // Check there is at least 1 cluster and that the robot cloud is published
  if ((cluster_indices.size() < 1) || (pc_in->points.size() < 1))
    return;
  
  double min_dist;
  std::vector<double> min_dists;
  std::vector<int> cluster_id;
  typename pcl::PointCloud<PointT>::Ptr temp_pc(new pcl::PointCloud<PointT>);
  typename std::vector< pcl::PointCloud<PointT> > temp_pcs;
  std::vector< geometry_msgs::PointStamped> cluster_pt_mins, pc_pt_mins;
  for(int i=0; i<cluster_indices.size();i++){
    cluster_id.clear();
    cluster_id.push_back(i);
    
    pc_extract_clusters(clustered_pc, cluster_indices, cluster_id, temp_pc);
    pc_to_pc_min_dist(temp_pc, pc_in, min_dist, cluster_pt_min, pc_pt_min);
    
    min_dists.push_back(min_dist);
    temp_pcs.push_back(*temp_pc);
    cluster_pt_mins.push_back(cluster_pt_min);
    pc_pt_mins.push_back(pc_pt_min);
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
  cluster_pt_min = cluster_pt_mins[min_id];
  pc_pt_min = pc_pt_mins[min_id];
  
}

#endif
