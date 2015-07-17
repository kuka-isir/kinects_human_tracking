#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

using namespace std;
using namespace boost::accumulators;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;
typedef pcl::PointXYZ PXYZ;
typedef pcl::PointCloud<PXYZ> PCXYZ;

class BackgroundStore
{
public:
  BackgroundStore(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh)
  {    
    n_frames_ = 0;

    nh_priv.getParam("pc_topic", pc_topic_);
    nh_priv.getParam("bg_frames", bg_frames_);
    nh_priv.getParam("min_file", min_file_);
    nh_priv.getParam("max_file", max_file_);
    
    pc_sub_ = nh.subscribe<PCRGB>(pc_topic_, 1, &BackgroundStore::recvPCCallback, this);
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);
  void gen_bg(); //Store the generated stats as bg-images
  void take_bg_stats(); //Keep statistics on the point clouds
  vector<accumulator_set< float, stats<tag::mean, tag::variance, tag::max, tag::min> > > acc_vec;

  PCXYZ pc_min_, pc_max_; // Range of values that belong to the background
  PCRGB pc_in_;
  size_t n_frames_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  string pc_topic_, min_file_, max_file_;
  int bg_frames_;
};

void BackgroundStore::recvPCCallback(const PCRGB::ConstPtr& msg)
{
  if (n_frames_>bg_frames_){
    cout << "Done with frames. End it." << endl;
    return;
  }
  else{
    pc_in_ = *msg;
    take_bg_stats();
  }
}

void BackgroundStore::gen_bg()
{
  pcl::copyPointCloud(pc_in_, pc_max_);
  pcl::copyPointCloud(pc_in_, pc_min_);  
  float p1 = 0.053353;
  float p2 = 0.0012123;

  for(size_t i=0; i<acc_vec.size(); i++){
    float pmean = boost::accumulators::mean(acc_vec[i]);
    float pstd = pow(boost::accumulators::variance(acc_vec[i]), 1./2.);
    float minz = boost::accumulators::min(acc_vec[i]);
    float maxz = boost::accumulators::max(acc_vec[i]);

    float diff_minz = pow(p1 * minz + p2, 2);
    float diff_maxz = pow(p1 * maxz + p2, 2);
    pc_max_[i].z = maxz + diff_maxz*1.5;
    pc_min_[i].z = minz - diff_minz*1.5;
  }
  //store as PCD
  pcl::io::savePCDFile(min_file_, pc_min_);
  pcl::io::savePCDFile(max_file_, pc_max_);
}

void BackgroundStore::take_bg_stats()
{
  if (n_frames_==0) //first frame
    acc_vec.resize(pc_in_.size());

  cout << "PC Size = " << pc_in_.size() << endl;

  for(size_t i=0; i< pc_in_.size(); ++i){
    PRGB pt = pc_in_[i];
    if(!isnan(pt.z))
      acc_vec[i](pt.z);
    }
  
    n_frames_++;
    cout << "Frame No. = " << n_frames_ << endl;
    if (n_frames_>bg_frames_){
      gen_bg();
  }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "bg_store");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    BackgroundStore bg_store(nh, nh_priv);
    ros::spin();
    return 0;
}
