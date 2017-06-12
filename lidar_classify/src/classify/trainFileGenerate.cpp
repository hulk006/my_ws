#include <ros/ros.h>
#include "trackManager.h"
#include "FeatureManager.h"
#include "std_msgs/String.h"
using namespace Robosense;

int control_mode = 1;
int Send_signal=0;
void controlCallback(const std_msgs::String::ConstPtr& msg)
{
    control_mode = 1;
    Send_signal =1;
    ROS_INFO("qq");
}
void continaulCallback(const std_msgs::String::ConstPtr& msg)
{
    control_mode = 1;
    Send_signal =1;
    ROS_INFO("123");
}

void initMarker(visualization_msgs::Marker& marker)
{
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shape_rect";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.id = 1;

    marker.pose.position.x=0;
    marker.pose.position.y=0;
    marker.pose.position.z=0;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "classify_roi_svm");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  ros::Rate loop_rate(1);
  std::string trackfile_ ;//= "/home/fan/unlabeled.txt"
  std::string filenameSave_;
  int label;
  private_nh.param("trackfile", trackfile_ , std::string("/home/xcon/data/trainData/testpc.txt"));
  private_nh.param("filenameSave", filenameSave_ , std::string("/home/xcon/data/trainData/testcarpedbkg.txt"));

  TrackManager tm(trackfile_);
  std::cout << "Loaded " << tm.tracks_.size() << " tracks." << std::endl;
  std::cout << std::endl;
  for(int track_count = 0; track_count<tm.tracks_.size(); track_count++)
  {
      if(tm.tracks_[track_count]->label_ == "car")
          label = 1;
      else if(tm.tracks_[track_count]->label_ == "truck")
          label = 2;
      else if(tm.tracks_[track_count]->label_ == "pedestrian" || tm.tracks_[track_count]->label_ == "bicycle")
          label = 3;
      else if(tm.tracks_[track_count]->label_ == "background")
          label = 4;
      if(label == 0)
          continue;
      for(int seg_count = 0; seg_count < tm.tracks_[track_count]->segments_.size(); seg_count++)
      {
          std::vector<int> choosed_feature;
          choosed_feature.push_back(-1);
          FeatureManager fm(tm.tracks_[track_count]->segments_[seg_count]->cloud_,label);
          fm.outFileSVMSamples(filenameSave_,choosed_feature);
      }
  }
  return 0;
}
