#include <kinects_human_tracking/kinect_img_bg_store.hpp>

class BackgroundImageStore
{
public: 
  BackgroundImageStore(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  {    
    n_frames_ = 0;
    bg_generated_ = false;
    
    string img_topic;
    nh_priv.getParam("img_topic", img_topic);
    nh_priv.getParam("bg_frames", bg_frames_);
    
    img_sub_ = nh.subscribe<sensor_msgs::Image>(img_topic, 1, &BackgroundImageStore::callback, this);
    service_ = nh_priv.advertiseService("get_background", &BackgroundImageStore::get_background, this);
    
  }
  
protected:
  void callback(const sensor_msgs::Image::ConstPtr& msg);
  void gen_bg(); //Store the generated stats as bg-images
  bool get_background(kinects_human_tracking::GetBackground::Request &req, kinects_human_tracking::GetBackground::Response &res);
  
  vector<accumulator_set< float, stats<tag::mean, tag::variance, tag::max, tag::min> > > acc_vec;

  ros::Subscriber img_sub_;
  ros::ServiceServer service_;
  size_t n_frames_;
  sensor_msgs::Image img_in_, img_min_;
  int bg_frames_;
  bool bg_generated_;
};

void BackgroundImageStore::callback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (n_frames_>bg_frames_){
    
    if (!bg_generated_){
      ROS_INFO("Done loading the background. Publishing...");
      gen_bg();
    }
    
    return;
  }
  else{
    img_in_ = *msg;
    if (n_frames_==0)
      acc_vec.resize(img_in_.data.size());
    
    for(size_t i=0; i< acc_vec.size(); ++i){
      float pixel = img_in_.data[i];
      if(!isnan(pixel))
	acc_vec[i](pixel);
    }  
    
    n_frames_++;
    std::cout<< "Frame nb : "<<n_frames_<<std::endl;
  }
   
}

bool BackgroundImageStore::get_background(kinects_human_tracking::GetBackground::Request  &req, kinects_human_tracking::GetBackground::Response &res)
{
  res.background = img_min_;
  res.success = true;
  return true;
}

void BackgroundImageStore::gen_bg()
{
  cv_bridge::CvImagePtr min_mat = cv_bridge::toCvCopy(img_in_);

  for(size_t i=0; i<acc_vec.size(); i++){
    float pmin = boost::accumulators::min(acc_vec[i]);
    min_mat->image.data[i] = pmin ;
  } 
  
  img_min_ = *(min_mat->toImageMsg());

  bg_generated_ = true;
    
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "bg_store");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    BackgroundImageStore bg_store(nh, nh_priv);
    ros::spin();
    return 0;
}
