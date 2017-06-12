//
// Created by mjj on 17-5-27.
//

#ifndef PROJECT_DRAWRVIZ_H
#define PROJECT_DRAWRVIZ_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include "Tracking.h"

class drawRviz
{
public:
    drawRviz();
    ~drawRviz(){};

    void drawTrack(const ros::Publisher& vel_pub, const ros::Publisher& pc_pub, const std_msgs::Header _header,
                    const std::vector<precision_tracking::Tracker>& trackers);
private:
    void drawRect(visualization_msgs::Marker &cuboid_marker, const precision_tracking::Tracker& singleTracker);
    void pushBackCubePoints(visualization_msgs::Marker &cuboid_marker, geometry_msgs::Point cuboid[]);

    template<typename T>
    std::string num2str(T num)
    {
        std::stringstream ss;
        std::string st;
        ss << num;
        ss >> st;
        return st;
    }

    std::vector<cv::Scalar> _colors;
    int pubNum = 100;

};

#endif //PROJECT_DRAWRVIZ_H
