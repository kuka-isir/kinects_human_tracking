#include <kinects_human_tracking/merge_kinects_pc.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "merge_kinects_pc");
  ros::NodeHandle nh, nh_priv("~");
  
  first_frame_ = true;
  
  // Get params topics and frames names
  int nb_topics = 0;
  bool done_loading;
  string topic_name, temp;
  cout << "Merging : " << topic_name <<endl;
  do{
    topic_name = "topic_name"+ boost::lexical_cast<string>(nb_topics+1);    
    done_loading = (!nh_priv.getParam(topic_name, temp)) || (temp=="") ;
    
    if(!done_loading){
      in_topic_names_.push_back(temp);
      cout << temp <<endl;
      nb_topics++;
    }
    
  }while(!done_loading);  
  nh_priv.getParam("out_frame", out_frame_);
  nh_priv.getParam("out_topic_name", out_topic_name_);
  cout << "Merging "<<nb_topics<<" topics to topic "<<out_topic_name_<<" in frame "<<out_frame_<<endl;
  
  transforms_.resize(nb_topics);
  
  // Initialize shared pointers and reserve memory
  for (int i=0; i<nb_topics;i++){
    pcs_.push_back(boost::shared_ptr<PointCloudSM>(new PointCloudSM));
    pcs_[i]->reserve(10000);
  }
  merged_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM); 
  merged_pc_->reserve(10000);
  
  // Ros Subscribers and Publishers
  pc_pub_ = nh.advertise<PointCloudSM>(out_topic_name_, 1);
  
  vector<boost::shared_ptr<message_filters::Subscriber<PCMsg> > > subs(nb_topics);
  for (int i=0; i<nb_topics; i++)
    subs[i] = boost::shared_ptr<message_filters::Subscriber<PCMsg> >(new message_filters::Subscriber<PCMsg>(nh, in_topic_names_[i],1));
  
  switch( nb_topics ){
    case 0: 
      ROS_ERROR("No topics to merge provided");
      return 1; break;
    case 1:{
      ros::Subscriber pc_sub = nh.subscribe<PCMsg>(in_topic_names_[0], 1, callback);
      ros::spin();
      break;
    }
    case 2:{ 
      message_filters::Synchronizer<MySyncPolicy2> sync(MySyncPolicy2(10), *subs[0], *subs[1]);
      sync.registerCallback(boost::bind(&callback2, _1, _2));
      cout <<"callback2"<<endl;
      ros::spin();
      break;
    }
    case 3:{
      message_filters::Synchronizer<MySyncPolicy3> sync(MySyncPolicy3(10), *subs[0], *subs[1], *subs[2]);
      sync.registerCallback(boost::bind(&callback3, _1, _2, _3));
      ros::spin();
      break;
    }
    case 4:{
      message_filters::Synchronizer<MySyncPolicy4> sync(MySyncPolicy4(10), *subs[0], *subs[1], *subs[2], *subs[3]);
      sync.registerCallback(boost::bind(&callback4, _1, _2, _3, _4));
      ros::spin();
      break;
    }
    case 5:{
      message_filters::Synchronizer<MySyncPolicy5> sync(MySyncPolicy5(10), *subs[0], *subs[1], *subs[2], *subs[3], *subs[4]);
      sync.registerCallback(boost::bind(&callback5, _1, _2, _3, _4, _5));
      ros::spin();
      break;
    }
    case 6:{
      message_filters::Synchronizer<MySyncPolicy6> sync(MySyncPolicy6(10), *subs[0], *subs[1], *subs[2], *subs[3], *subs[4], *subs[5]);
      sync.registerCallback(boost::bind(&callback6, _1, _2, _3, _4, _5, _6));
      ros::spin();
      break;
    }
    case 7:{
      message_filters::Synchronizer<MySyncPolicy7> sync(MySyncPolicy7(10), *subs[0], *subs[1], *subs[2], *subs[3], *subs[4], *subs[5], *subs[6]);
      sync.registerCallback(boost::bind(&callback7, _1, _2, _3, _4, _5, _6, _7));
      ros::spin();
      break;
    }
    case 8:{
      message_filters::Synchronizer<MySyncPolicy8> sync(MySyncPolicy8(10), *subs[0], *subs[1], *subs[2], *subs[3], *subs[4], *subs[5], *subs[6], *subs[7]);
      sync.registerCallback(boost::bind(&callback8, _1, _2, _3, _4, _5, _6, _7, _8));
      ros::spin();
      break;
    }
    case 9:{
      message_filters::Synchronizer<MySyncPolicy9> sync(MySyncPolicy9(10), *subs[0], *subs[1], *subs[2], *subs[3], *subs[4], *subs[5], *subs[6], *subs[7], *subs[8]);
      sync.registerCallback(boost::bind(&callback9, _1, _2, _3, _4, _5, _6, _7, _8, _9));
      ros::spin();
      break;
    }
    default:
      ROS_ERROR("Too many topics provided. Current limitation is of 9 kinects");
      return 1;
      break;
  }
  
  return 0;
}

void callback(const PCMsg::ConstPtr& first_msg_pc2){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}

void callback2(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}

void callback3(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}
void callback4(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  pcs_msg_pc2_.push_back(fourth_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}
void callback5(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  pcs_msg_pc2_.push_back(fourth_msg_pc2);
  pcs_msg_pc2_.push_back(fifth_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}
void callback6(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  pcs_msg_pc2_.push_back(fourth_msg_pc2);
  pcs_msg_pc2_.push_back(fifth_msg_pc2);
  pcs_msg_pc2_.push_back(sixth_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}
void callback7(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2, const PCMsg::ConstPtr& seventh_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  pcs_msg_pc2_.push_back(fourth_msg_pc2);
  pcs_msg_pc2_.push_back(fifth_msg_pc2);
  pcs_msg_pc2_.push_back(sixth_msg_pc2);
  pcs_msg_pc2_.push_back(seventh_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}
void callback8(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2, const PCMsg::ConstPtr& seventh_msg_pc2, const PCMsg::ConstPtr& eighth_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  pcs_msg_pc2_.push_back(fourth_msg_pc2);
  pcs_msg_pc2_.push_back(fifth_msg_pc2);
  pcs_msg_pc2_.push_back(sixth_msg_pc2);
  pcs_msg_pc2_.push_back(seventh_msg_pc2);
  pcs_msg_pc2_.push_back(eighth_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}
void callback9(const PCMsg::ConstPtr& first_msg_pc2, const PCMsg::ConstPtr& second_msg_pc2, const PCMsg::ConstPtr& third_msg_pc2, const PCMsg::ConstPtr& fourth_msg_pc2, const PCMsg::ConstPtr& fifth_msg_pc2, const PCMsg::ConstPtr& sixth_msg_pc2, const PCMsg::ConstPtr& seventh_msg_pc2, const PCMsg::ConstPtr& eighth_msg_pc2, const PCMsg::ConstPtr& ninth_msg_pc2 ){
  pcs_msg_pc2_.clear();
  pcs_msg_pc2_.push_back(first_msg_pc2);
  pcs_msg_pc2_.push_back(second_msg_pc2);
  pcs_msg_pc2_.push_back(third_msg_pc2);
  pcs_msg_pc2_.push_back(fourth_msg_pc2);
  pcs_msg_pc2_.push_back(fifth_msg_pc2);
  pcs_msg_pc2_.push_back(sixth_msg_pc2);
  pcs_msg_pc2_.push_back(seventh_msg_pc2);
  pcs_msg_pc2_.push_back(eighth_msg_pc2);
  pcs_msg_pc2_.push_back(ninth_msg_pc2);
  merge_pcs(pcs_msg_pc2_);
}

void merge_pcs(vector<PCMsg::ConstPtr> pcs_msg_pc2){
  
  for(int i=0; i<pcs_msg_pc2.size(); i++){
    // conversionns to pcl::PointCloud
    pcl::fromROSMsg(*pcs_msg_pc2[i], *pcs_[i]);
    
    if(first_frame_)
      // get transform between the pc frame and the output frame
      get_transform(pcs_[i]->header.frame_id, transforms_[i]);
    
    // transform the pc to the output frame
    pcl::transformPointCloud(*pcs_[i], *pcs_[i], transforms_[i].translation, transforms_[i].rotation);
    pcs_[i]->header.frame_id = out_frame_;
    
    // sum the pcs
    if(i==0)
      *merged_pc_ = *pcs_[i];
    else
      *merged_pc_ += *pcs_[i];
  }

  first_frame_ = false;
  pc_pub_.publish(*merged_pc_);
  
}

void get_transform(string pc_frame, pclTransform &pc_transform){
   
  tf::TransformListener trans_listener;
  tf::StampedTransform stampedTransform;
  
  try{
    trans_listener.waitForTransform(out_frame_, pc_frame, ros::Time(0), ros::Duration(10.0));
    trans_listener.lookupTransform(out_frame_, pc_frame, ros::Time(0), stampedTransform);
  }
  catch(tf::TransformException &ex){
    cout << ex.what() << endl;
    ROS_ERROR("%s", ex.what());
  }
  tf::vectorTFToEigen(stampedTransform.getOrigin(), pc_transform.translation);      
  tf::quaternionTFToEigen(stampedTransform.getRotation(), pc_transform.rotation);

  return;
}