//
// Created by mjj on 17-5-2.
//

#include <ros/ros.h>
#include "lidarCluster.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    lidar_cluster clu(node,private_nh);

    ros::spin();

    return 0;
}