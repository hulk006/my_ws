#include <ros/ros.h>
#include "classifySVM.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "classify_roi_svm");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  ClassifySVM mf(node, priv_nh);

  ros::spin();




    //pub_test = node.advertise<sensor_msgs::PointCloud2>("test",10);
    pub_seg = node.advertise<sensor_msgs::PointCloud2>("segWithGrid", 10);
    pub_segResult = node.advertise<lidar_seg_grid::SegResultDetect>("segResult",10);
    ros::Subscriber sub=node.subscribe("groundDetectResult" , 10 , segCallback); //队列大小，以防我们处理消息的速度不够快，在缓存了10个消息后，再有新的消息到来就将开始丢弃先前接收的消息。





  return 0;
}
