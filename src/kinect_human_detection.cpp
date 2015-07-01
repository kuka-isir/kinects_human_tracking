//pcl includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_plane.h>

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
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

/**
   Subscribe to a pointCloud and figure if there is 
   a human inside. Then compute the distance between
   the human and the robot using the robot's pointcloud
**/

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;
typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg> MySyncPolicy;

typedef Eigen::Vector3f ClusterPoint;
struct ClusterStats{
  ClusterPoint mean;
  ClusterPoint var; 
  ClusterPoint min; 
  ClusterPoint max; 
  ClusterPoint median;
};
typedef struct ClusterStats ClusterStats;

void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg);
// void minDistThread (int indice_min, int indice_max,int &min_idx,int &min_jdx, float &min_dist);

PointCloudSM::Ptr pcl_pc1, clustered_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr robot_pc;
ros::Publisher human_pc_pub, pc_clustered_pub, cloud_mini_pt_pub, dist_pt_pub;
tf::TransformListener *tf_listener; 

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_human_detection");
  ros::NodeHandle nh, nh_priv("~");
  tf_listener = new tf::TransformListener();
  sleep(0.5); //to make sure tf_listener is ready
   
  // Get params topics and frames names
  string kinect_topic_name, robot_topic_name, clusters_topic_name, out_topic_name;
  nh_priv.getParam("kinect_topic_name",kinect_topic_name);
  nh_priv.getParam("robot_topic_name",robot_topic_name);
  nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  nh_priv.getParam("out_topic_name",out_topic_name);
  
  // Initialize PointClouds
  pcl_pc1 = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  robot_pc = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
  clustered_cloud = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  pcl_pc1->reserve(10000);
  robot_pc->reserve(10000);
  clustered_cloud->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_clustered_pub = nh.advertise<PointCloudSM>(clusters_topic_name, 1);
  human_pc_pub = nh.advertise<PointCloudSM>(out_topic_name, 1);
  cloud_mini_pt_pub = nh.advertise<geometry_msgs::PointStamped>("kinect_both/min_human_pt",1);
  dist_pt_pub = nh.advertise<geometry_msgs::PointStamped>("kinect_both/min_robot_pt",1);
  
  message_filters::Subscriber<PCMsg> kinect_pc_sub(nh, kinect_topic_name, 1);
  message_filters::Subscriber<PCMsg> robot_pc_sub(nh, robot_topic_name, 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), kinect_pc_sub, robot_pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& human_pc_msg, const PCMsg::ConstPtr& robot_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*human_pc_msg, *pcl_pc1);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*pcl_pc1, *pcl_pc1, indices);
  
  /* Removing points on the floor, here below height y=0
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr height_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  height_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 0.0)));
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (height_cond);
  condrem.setInputCloud (pcl_pc1);
  condrem.setKeepOrganized(true);
  condrem.filter (*pcl_pc1);
  //*/
  
  // Downsampling of the kinects cloud
  float voxel_size=0.01;
  int max_cluster_size = 250000;
  int min_cluster_size = 100;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  vox_grid.setInputCloud(pcl_pc1);
  vox_grid.setLeafSize(voxel_size,voxel_size,voxel_size);
  vox_grid.filter(*pc_filtered);
  
  /* Ground removal:
  const Eigen::VectorXf ground_coeffs;
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> > ground_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(pc_filtered));
  ground_model->selectWithinDistance(ground_coeffs, 1.5*voxel_size, *inliers);
  PointCloudSM::Ptr no_ground_cloud(new PointCloudSM);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(pc_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud);  
  //*/
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (pc_filtered);
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (2*voxel_size); // 2cm
  ec.setMinClusterSize (min_cluster_size);
  //ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pc_filtered);
  ec.extract (cluster_indices);
    
  std::vector<double> cluster_heights;
  for(int i = 0; i<cluster_indices.size(); i++){
    int cluster_size = cluster_indices[i].indices.size();
    if (i==0){
      for(int j=0; j<cluster_size;j++){
	  pc_filtered->points[cluster_indices[i].indices[j]].r = 255;
	  pc_filtered->points[cluster_indices[i].indices[j]].g = 0;
	  pc_filtered->points[cluster_indices[i].indices[j]].b = 0;
      }
    }
    if (i==1){
      for(int j=0; j<cluster_size;j++){
	  pc_filtered->points[cluster_indices[i].indices[j]].r = 0;
	  pc_filtered->points[cluster_indices[i].indices[j]].g = 255;
	  pc_filtered->points[cluster_indices[i].indices[j]].b = 0;
      }
    }
    if (i==2){
      for(int j=0; j<cluster_size;j++){
	  pc_filtered->points[cluster_indices[i].indices[j]].r = 0;
	  pc_filtered->points[cluster_indices[i].indices[j]].g = 0;
	  pc_filtered->points[cluster_indices[i].indices[j]].b = 255;
      }
    }

    // Stats on clusters
    boost::accumulators::accumulator_set< float, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance, boost::accumulators::tag::min, boost::accumulators::tag::max, boost::accumulators::tag::median> > x_acc, y_acc, z_acc; 
    for(vector<int>::const_iterator pint = cluster_indices[i].indices.begin(); pint!=cluster_indices[i].indices.end(); ++pint){
      pcl::PointXYZRGB p = pc_filtered->points[*pint];
      x_acc(p.x);
      y_acc(p.y);
      z_acc(p.z);
    }
    ClusterStats cluster_stats; //Stats for one cluster
    cluster_stats.mean = ClusterPoint(boost::accumulators::mean(x_acc), boost::accumulators::mean(y_acc), boost::accumulators::mean(z_acc));
    cluster_stats.var = ClusterPoint(boost::accumulators::variance(x_acc), boost::accumulators::variance(y_acc), boost::accumulators::variance(z_acc));
    cluster_stats.min = ClusterPoint(boost::accumulators::min(x_acc), boost::accumulators::min(y_acc), boost::accumulators::min(z_acc));    
    cluster_stats.max = ClusterPoint(boost::accumulators::max(x_acc), boost::accumulators::max(y_acc), boost::accumulators::max(z_acc));    
    cluster_stats.median = ClusterPoint(boost::accumulators::median(x_acc), boost::accumulators::median(y_acc), boost::accumulators::median(z_acc));    
    
    std::string tmp = boost::lexical_cast<std::string>(cluster_stats.max-cluster_stats.min);
    double cluster_height = (double)atof(tmp.c_str());
    cluster_heights.push_back(cluster_height);
  }
  pc_clustered_pub.publish(*pc_filtered);
  
  // Selecting only the clusters with height superior to 0.5
  int j = -1, nb_clusters_left = 0;
  *clustered_cloud=*pc_filtered;
  clustered_cloud->points.clear();
  clustered_cloud->width = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    j++;
    if(cluster_heights[j]<0.5)
      continue;
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      clustered_cloud->points.push_back(pc_filtered->points[*pit]); //*
      clustered_cloud->width++;
    }
    nb_clusters_left++;
  }
  clustered_cloud->width = clustered_cloud->points.size();
  clustered_cloud->height = 1;
  clustered_cloud->is_dense = true;
  human_pc_pub.publish(*clustered_cloud); 

  /* Computation of minimum distance between human cloud and a point
  Eigen::Vector4f pivot_pt(2,1,2,1);
  if (nb_clusters_left == 1){
    
    last = ros::Time::now();
    
    float dist, min_dist = 100000000;
    int min_idx = 0;
    for (size_t i = 0; i < clustered_cloud->points.size (); ++i){
      pcl::Vector4fMap pt = clustered_cloud->points[i].getVector4fMap();
      dist = (pivot_pt - pt).norm ();
      if (dist < min_dist){
	min_idx = i;
	min_dist = dist;
      }
    }
    cout<< "min_dist : "<<min_dist<<endl;
   
    end = ros::Time::now();
    dt = end-last;
    cout << dt.toSec()<<" seconds to compute closest point"<<endl;
    
    geometry_msgs::PointStamped pt, pt_mini;
    pt.header.frame_id = clustered_cloud->header.frame_id;
    pt_mini.header.frame_id = clustered_cloud->header.frame_id;
    pt.point.x = 2;
    pt.point.y = 1;
    pt.point.z = 2;
    pt_mini.point.x = clustered_cloud->points[min_idx].x;
    pt_mini.point.y = clustered_cloud->points[min_idx].y;
    pt_mini.point.z = clustered_cloud->points[min_idx].z;
    
    cloud_mini_pt_pub.publish<geometry_msgs::PointStamped>(pt_mini);
    dist_pt_pub.publish<geometry_msgs::PointStamped>(pt);
  }
  //*/ 
     
  
  if (nb_clusters_left == 1){
    // Transform robot cloud in the human frame
    sensor_msgs::PointCloud2 pcl_out; 
    tf_listener->waitForTransform(clustered_cloud->header.frame_id, robot_pc_msg->header.frame_id, robot_pc_msg->header.stamp, ros::Duration(10.0)); 
    pcl_ros::transformPointCloud(clustered_cloud->header.frame_id, *robot_pc_msg, pcl_out, *tf_listener); 
    pcl::fromROSMsg(pcl_out, *robot_pc);
   
    // Downsample the robot's cloud
    float voxel_size = 0.06;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(robot_pc);
    vox_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox_grid.filter(*robot_pc);
    
    // Computation of minimum distance between the human and the robot
    /*  Threaded version
    int id1, id2, jd1, jd2;
    float min_dist1, min_dist2;
    
    int robot_nb_pts = robot_pc->points.size();
    if(robot_nb_pts %2 ==0){
      boost::thread first_half_min_dist(&minDistThread, 0,robot_nb_pts/2,id1, jd1, min_dist1);
      boost::thread second_half_min_dist(&minDistThread, robot_nb_pts/2,robot_nb_pts,id2, jd2, min_dist2);
      first_half_min_dist.join();
      second_half_min_dist.join();
    }
    else{
      boost::thread first_half_min_dist(&minDistThread, 0,(robot_nb_pts-1)/2,id1, jd1, min_dist1);
      boost::thread second_half_min_dist(&minDistThread, (robot_nb_pts-1)/2 +1,robot_nb_pts,id2, jd2, min_dist2);
      first_half_min_dist.join();
      second_half_min_dist.join();
    }
    
    int min_idx, min_jdx;
    float min_dist;
    if (min_dist1<min_dist2){
      min_jdx = jd1;
      min_idx = id1;
      min_dist = min_dist1;
    }
    else{
      min_jdx = jd2;
      min_idx = id2;
      min_dist = min_dist2;
    }
    //*/
    
    //* Non-Threaded
    float dist = 0, min_dist = 100000000;
    int min_idx = 0;
    int min_jdx = 0;
    for (size_t i = 0; i < clustered_cloud->points.size (); ++i){
      for (size_t j = 0; j < robot_pc->points.size (); ++j){  
	pcl::Vector4fMap pt = clustered_cloud->points[i].getVector4fMap();
	pcl::Vector4fMap robot_pt = robot_pc->points[j].getVector4fMap();
	dist = (robot_pt - pt).norm ();
	if (dist < min_dist){
	  min_idx = i;
	  min_jdx = j;
	  min_dist = dist;
	}
      }
    }
    //*/
    
    // Publish closest points between human and robot
    geometry_msgs::PointStamped pt, pt_mini;
    pt.header.frame_id = robot_pc->header.frame_id;
    pt_mini.header.frame_id = clustered_cloud->header.frame_id;
    pt_mini.point.x = clustered_cloud->points[min_idx].x;
    pt_mini.point.y = clustered_cloud->points[min_idx].y;
    pt_mini.point.z = clustered_cloud->points[min_idx].z;
    pt.point.x = robot_pc->points[min_jdx].x;
    pt.point.y = robot_pc->points[min_jdx].y;
    pt.point.z = robot_pc->points[min_jdx].z;
    cloud_mini_pt_pub.publish<geometry_msgs::PointStamped>(pt_mini);
    dist_pt_pub.publish<geometry_msgs::PointStamped>(pt);
  }
}

/* Function used for threaded version of cloud to cloud distance computation
void minDistThread (int indice_min, int indice_max,int &min_idx,int &min_jdx, float &min_dist){
  
  ros::Time before = ros::Time::now();
  float dist;
  for (size_t i = 0; i < clustered_cloud->points.size (); ++i){
    for (size_t j = indice_min; j < indice_max; ++j){  
      pcl::Vector4fMap pt = clustered_cloud->points[i].getVector4fMap();
      pcl::Vector4fMap robot_pt = robot_pc->points[j].getVector4fMap();
      dist = (robot_pt - pt).norm ();
      if (dist < min_dist){
	min_idx = i;
	min_jdx = j;
	min_dist = dist;
      }
    }
  }
  ros::Time after = ros::Time::now();
  ros::Duration dt = after - before ;
  cout << "Half computation time is : "<<dt.toSec()<<endl;
}*/
