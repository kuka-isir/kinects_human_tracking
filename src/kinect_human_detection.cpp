#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/transforms.h>
#include <opencv2/opencv.hpp>
#include <boost/timer.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/people/ground_based_people_detection_app.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/people/person_cluster.h>


//ros-includes
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

/**
   Subscribe a pointCloud and figure if there is 
   a human inside.
**/

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;

void callback(const PCMsg::ConstPtr& front_pc);

void find_ppl_clusters(const PointCloudSM::Ptr cloud, 
		  vector<pcl::PointIndices>& init_indices, 
		  std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB> >& clusters,
				      const Eigen::VectorXf ground_coeffs,
				      const int max_c_size,
				      const int min_c_size);

PointCloudSM::Ptr pcl_pc1;
ros::Publisher pc_pub, pc_w_ground_pub;
ros::Subscriber pc_sub;
pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
pcl::PointXYZRGB newpoint;

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_human_detection");
  ros::NodeHandle nh, nh_priv("~");
  
  // Get params topics and frames names
  string in_topic_name, w_ground_topic_name, out_topic_name;
  nh_priv.getParam("in_topic_name",in_topic_name);
  nh_priv.getParam("w_ground_topic_name",w_ground_topic_name);
  nh_priv.getParam("out_topic_name",out_topic_name);
  
  // Initialize PointClouds
  pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Ros Subscribers and Publishers
  pc_sub = nh.subscribe<PCMsg>(in_topic_name, 1, callback);
  pc_w_ground_pub = nh.advertise<PointCloudSM>(w_ground_topic_name, 1);
  pc_pub = nh.advertise<PointCloudSM>(out_topic_name, 1);
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& msg_pc2_){
  
  // conversionns to pcl::PointCloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg_pc2_, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pc1);
  
  ros::Time begin = ros::Time::now();
  
  /* DEPRECATED
  pcl::PointIndicesPtr indices_in (new pcl::PointIndices);
  for(size_t i=0; i<pcl_pc1->size(); ++i){
    if (pcl_pc1->at(i).y <0)
      indices_in->indices.push_back(i);
  }
  
  pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true); // Initializing with true will allow us to extract the removed indices
  eifilter.setInputCloud (pcl_pc1);
  eifilter.setIndices (indices_in);
  eifilter.filter (*pcl_pc1);
  //*/
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*pcl_pc1, *pcl_pc1, indices);
  
  // Remove points on the floor, here below height y=0
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr height_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  height_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 0.0)));
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (height_cond);
  condrem.setInputCloud (pcl_pc1);
  condrem.setKeepOrganized(true);
  condrem.filter (*pcl_pc1);
  
  // Remove statistical ouliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (pcl_pc1);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*pcl_pc1);
  
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*pcl_pc1, *pcl_pc1, indices);
  // Remove points with not enough neighbours
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  outrem.setInputCloud(pcl_pc1);
  outrem.setRadiusSearch(0.5); 
  outrem.setMinNeighborsInRadius (10);
  outrem.filter (*pcl_pc1);
  
  
  cout << "Cloud has " << pcl_pc1->points.size() <<" points" << endl;
  pc_w_ground_pub.publish(*pcl_pc1);
  
  
  float voxel_size=0.01;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  vox_grid.setInputCloud(pcl_pc1);
  vox_grid.setLeafSize(voxel_size,voxel_size,voxel_size);
  vox_grid.filter(*pc_filtered);
  cout << "Cloud after filtering has " << pc_filtered->points.size() <<" points" << endl;
  
  const Eigen::VectorXf ground_coeffs;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (pc_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pc_filtered);
  ec.extract (cluster_indices);
  
  cout << "No. of clusters: " << cluster_indices.size() << endl;
  
/*
  std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB> > ppl_clusters;
  find_ppl_clusters(pc_filtered,
  		    cluster_indices,
  		    ppl_clusters,
  		    ground_coeffs,
		    max_cluster_size,
		    min_cluster_size);

  //debug
  cout << "No. of clusters: " << cluster_indices.size() << endl;

  int j = 0;
  
  //replace cluster indices with people clusters
  int n_ppl=0;
  cluster_indices.clear();
  for(std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB> >::iterator 
  	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
      {
  	cluster_indices.push_back(it->getIndices());
  	n_ppl++;
      }
  
  cout << "No. of people: " << n_ppl << endl; 
  //*/
  /*
  ros::Time end = ros::Time::now();
  ros::Duration diff = (end-begin);
  cout <<"Time "<< diff.toSec()<<endl;
  */
  
  //pc_w_ground_pub.publish(*pc_filtered);
}


void find_ppl_clusters(const PointCloudSM::Ptr cloud, 
		  vector<pcl::PointIndices>& init_indices, 
		  std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB> >& clusters,
				      const Eigen::VectorXf ground_coeffs,
				      const int max_c_size,
				      const int min_c_size)
{
  
  //debug
  // cout << "Started this finding.." << endl;

  float max_height_= 2.3; float min_height_= 1.2;
  int min_a_merge = 200; float max_dist_gr=0.4;
  bool camera_vertical=false, compute_head=false;

  float sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f
				(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();

  // Person clusters creation from clusters indices:
  for(std::vector<pcl::PointIndices>::const_iterator it = 
	init_indices.begin(); it != init_indices.end(); ++it)
    {
      pcl::people::PersonCluster<pcl::PointXYZRGB> 
	cluster(cloud, *it, ground_coeffs, sqrt_ground_coeffs_, compute_head, camera_vertical); // PersonCluster creation
      clusters.push_back(cluster);
    }
/*
  //debug
  // cout << "Created ppl clusters.." << endl;

  std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB> > new_clusters;
  for(unsigned int i = 0; i < clusters.size(); i++) // for every cluster
    {
      // if (clusters[i].getHeight() <= max_height_)
  	new_clusters.push_back(clusters[i]);
    }

  std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB> > ppl_filtered;
  // Remove clusters according to rules
  rm_ppl_clusters(cloud, clusters, new_clusters,
		  ground_coeffs, sqrt_ground_coeffs_, max_height_, min_height_,
		  max_dist_gr, max_c_size, min_c_size);

  ppl_filtered.clear();
  for(unsigned int i = 0; i < new_clusters.size(); i++) // for every cluster
    {
     
  	    ppl_filtered.push_back(new_clusters[i]);

    }

  // //debug
  // cout << "Donadone.." << endl;
  clusters.clear();
  clusters = ppl_filtered;
  */
  return;
}


















