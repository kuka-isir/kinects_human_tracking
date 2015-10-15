#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <ros/ros.h>

#include "kinects_human_tracking/GetBackground.h"

using namespace std;

class BackgroundImageSubtract
{
public:
  BackgroundImageSubtract(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : nh_(nh) {
    // Read stored min and max background
    string topic_in, topic_out, kinect_name, service_name;
    nh_priv.getParam("kinect_name", kinect_name);
    nh_priv.getParam("topic_in", topic_in);
    nh_priv.getParam("topic_out", topic_out); 
    
    service_name = kinect_name+"_img_bg_store/get_background";
    
    img_sub_ = nh.subscribe<sensor_msgs::Image>(topic_in, 1, &BackgroundImageSubtract::callback, this);
    img_pub_ = nh.advertise<sensor_msgs::Image>(topic_out, 1);
    
    client_ = nh.serviceClient<kinects_human_tracking::GetBackground>(service_name);
    kinects_human_tracking::GetBackground srv;
    client_.call(srv);
          
    img_min_ = boost::shared_ptr<sensor_msgs::Image> (new sensor_msgs::Image);
    *img_min_ = srv.response.background;
    
    ROS_INFO("Initialization done. Looping ...");
    
}
  
protected:
  void callback(const sensor_msgs::Image::ConstPtr& msg);

  sensor_msgs::Image::Ptr img_min_, img_max_, img_in_;
  cv_bridge::CvImage::Ptr min_cv_image_;
  ros::NodeHandle nh_;
  ros::Subscriber img_sub_;
  ros::Publisher img_pub_;
  ros::ServiceClient client_;
  cv_bridge::CvImage::Ptr in_cv_image, min_image;
  
};

void BackgroundImageSubtract::callback(const sensor_msgs::Image::ConstPtr& msg){
    
  in_cv_image = cv_bridge::toCvCopy(msg);
  min_image = cv_bridge::toCvCopy(*img_min_, "32FC1");

  cv::Mat out_img = in_cv_image->image.clone();
  
  for(int j=0; j<out_img.rows; j++){
    for(int i=0; i<out_img.cols;i++){
      float pix_in = float(in_cv_image->image.at<float>(cv::Point(i,j)));
      float pix_min = float(min_image->image.at<float>(cv::Point(i,j))/1000.0);
      if ((pix_min != 0) && (pix_in >= pix_min))
	out_img.at<float>(cv::Point(i,j))=0.0;     
    }
  }

  int morph_elem = 0;
  int morph_size = 2;
  int morph_operator = 0;
  cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  cv::morphologyEx(out_img, out_img, morph_operator, element);
  
  cv_bridge::CvImage out_ros_img;
  out_ros_img.encoding = in_cv_image->encoding;
  out_ros_img.header = in_cv_image->header;
  out_ros_img.image = out_img;
  out_ros_img.toImageMsg();
  img_pub_.publish(*out_ros_img.toImageMsg());
  
}


int main( int argc, char** argv ){
    ros::init(argc, argv, "bg_subtract");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    BackgroundImageSubtract bg_sub(nh, nh_priv);
    ros::spin();
    return 0;
}
