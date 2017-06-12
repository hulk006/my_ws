#include <ros/ros.h>
#include "xgbclassifyTrack.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seg_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  std::string pkgPath = ros::package::getPath("lidar_classify");

  xgbClassifyTrack xgbct(node,private_nh);
  ros::spin();
  return 0;
}
