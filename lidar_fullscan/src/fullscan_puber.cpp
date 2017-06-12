#include <ros/ros.h>
#include "modify_driver.h"
#include "convert_fullscan.h"
#include "std_msgs/String.h"


int control_mode;
int freq;
int Send_signal=0;
void controlCallback(const std_msgs::String::ConstPtr& msg)
{
  control_mode = 0;
  Send_signal =1;
  ROS_INFO("123");
}
void continaulCallback(const std_msgs::String::ConstPtr& msg)
{
  control_mode = 1;
  Send_signal =1;
  ROS_INFO("123");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fullscan_puber");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  private_nh.param("control_mode",control_mode,1);
  private_nh.param("frequency",freq,10);

  ros::Rate loop_rate(freq);

  ros::Subscriber control = node.subscribe("control_signal",1,controlCallback);
  ros::Subscriber control1 = node.subscribe("continual_signal",1,continaulCallback);

  pic_output = node.advertise<lidar_msg::VelodynePic>("rawdata_fullscan",10);
  pc_output = node.advertise<sensor_msgs::PointCloud2>("fullscan", 10);
  init_setup();
  // start the driver

  velodyne_driver::VelodyneDriver dvr(node, private_nh);
  // loop until shut down or end of file
  while(ros::ok())
  {
    if(control_mode == 0)
    {
      if(Send_signal == 1)
      {
        while(fullscan == 0)
        {
          dvr.poll();
        }
        fullscan = 0;
        Send_signal = 0;
        //loop_rate.sleep();
      }
    }
    else
    {
      while(fullscan == 0)
      {
        dvr.poll();
      }
      fullscan = 0;
      Send_signal = 0;
      loop_rate.sleep();
    }

    ros::spinOnce();
  }
  return 0;
}
