#include <stdio.h>
#include <iostream>

#include <kinects_human_tracking/pointCloudUtils.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;

class PcDownsampling
{
public:
  PcDownsampling(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : nh_(nh) {
    // Read stored min and max background
    string topic_in, topic_out;
    nh_priv.getParam("topic_in", topic_in);
    nh_priv.getParam("topic_out", topic_out);
    nh_priv.getParam("voxel_size", voxel_size_);

    pc_in_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
    pc_in_->reserve(10000);
    
    pc_sub_ = nh.subscribe<PointCloudSM>(topic_in, 1, &PcDownsampling::callback, this);
    pc_pub_ = nh.advertise<PointCloudSM>(topic_out, 1);
  }
  
protected:
  void callback(const PointCloudSM::ConstPtr& msg);

  PointCloudSM::Ptr pc_in_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
  double voxel_size_;

};

void PcDownsampling::callback(const PointCloudSM::ConstPtr& msg){
  
//   pc_downsampling(msg, voxel_size_, pc_in_);
  
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setInputCloud(msg);
  vox_grid.setLeafSize(voxel_size_,voxel_size_,voxel_size_);
  vox_grid.filter(*pc_in_);
   
  //publish subbed point cloud
  pc_pub_.publish(*pc_in_); // PC no longer organized
}

int main( int argc, char** argv ){
    ros::init(argc, argv, "bg_subtract");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    PcDownsampling bg_sub(nh, nh_priv);
    ros::spin();
    return 0;
}
