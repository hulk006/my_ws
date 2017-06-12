//
// Created by mjj on 17-5-2.
//

#include <ros/ros.h>
#include "../include/lidarTerrian.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terrian_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    terrian_estimate te(node,private_nh);

    ros::spin();

    return 0;
}