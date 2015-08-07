#include <stdio.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace std;

typedef pcl::PointXYZ PXYZ;
typedef pcl::PointCloud<PXYZ> PCXYZ;
typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

class BackgroundSubtract
{
public:
  BackgroundSubtract(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : nh_(nh) {
    // Read stored min and max background
    string min_file, max_file, topic_in, topic_out;
    nh_priv.getParam("min_file", min_file);
    nh_priv.getParam("max_file", max_file);
    nh_priv.getParam("topic_in", topic_in);
    nh_priv.getParam("topic_out", topic_out);
    nh_priv.getParam("voxel_size", voxel_size_);

    if(pcl::io::loadPCDFile<PXYZ> (min_file, pc_min_) == -1 )
      ROS_ERROR_STREAM("Min File not found. Path = " << min_file); 
    if(pcl::io::loadPCDFile<PXYZ> (max_file, pc_max_) == -1 )
      ROS_ERROR_STREAM("Max File not found. Path = " << max_file); 

    ROS_INFO("Loaded Min/Max files");
    pc_sub_ = nh.subscribe<PCRGB>(topic_in, 1, &BackgroundSubtract::recvPCCallback, this);
    pc_pub_ = nh.advertise<PCRGB>(topic_out, 1);
    bg_removed_ = boost::shared_ptr<PCRGB>(new PCRGB);
    bg_removed_->reserve(10000);
  }
  
protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);
  void backgroundSub(const PCRGB::ConstPtr& pc, pcl::PointIndices::Ptr &inds);

  PCXYZ pc_min_, pc_max_; // Range of values that belong to the background
  PCRGB::ConstPtr pc_in_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
  std::set<float> possible_z;
  PCRGB::Ptr bg_removed_;
  double voxel_size_;

};

void BackgroundSubtract::recvPCCallback(const PCRGB::ConstPtr& msg){
  pc_in_ = msg;

  pcl::PointIndices::Ptr inds(new pcl::PointIndices);
  
  // Subtract background
  backgroundSub(pc_in_, inds);

  
  pcl::ExtractIndices<PRGB> extract_filter;
  extract_filter.setNegative(false);
  extract_filter.setIndices(inds);
  extract_filter.setInputCloud(pc_in_);
  extract_filter.filter(*bg_removed_);
  
  pcl::VoxelGrid<PRGB> vox_grid;
  vox_grid.setInputCloud(bg_removed_);
  vox_grid.setLeafSize(voxel_size_,voxel_size_,voxel_size_);
  vox_grid.filter(*bg_removed_);
  
  //publish subbed point cloud
  pc_pub_.publish(*bg_removed_); // PC no longer organized
}

void BackgroundSubtract::backgroundSub(const PCRGB::ConstPtr& pc, pcl::PointIndices::Ptr &inds){
  //TODO: clip too far points 
  for(size_t i=0; i<pc->size(); i++){
    float p_min_z = pc_min_[i].z;
    float p_max_z = pc_max_[i].z;
    float pt_z = pc->at(i).z;

    if(!isnan(pt_z) && (isnan(p_min_z) || isinf(p_min_z) || (pt_z < p_min_z)))
      inds->indices.push_back(i);
    //if(isnan(pt_z) && ((isnan(p_max_z) || pt_z > p_max_z) || (isnan(p_min_z) || pt_z < p_min_z)))
  }
}

int main( int argc, char** argv ){
    ros::init(argc, argv, "bg_subtract");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    BackgroundSubtract bg_sub(nh, nh_priv);
    ros::spin();
    return 0;
}
