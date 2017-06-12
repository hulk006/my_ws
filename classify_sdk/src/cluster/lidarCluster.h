//
// Created by mjj on 17-5-2.
//

#ifndef LIDAR_CLUSTER_LIDARCLUSTER_H
#define LIDAR_CLUSTER_LIDARCLUSTER_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros/package.h"
#include <dirent.h>

#include "lidar_msg/cluster.h"
#include "lidar_msg/clusterArray.h"
#include "lidar_msg/terrian.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
#define BEAMS 16

#define PI 3.1415926535898

const float VERT_ANGLE2[] = {
        -0.2617993877991494,
        -0.22689280275926285,
        -0.19198621771937624,
        -0.15707963267948966,
        -0.12217304763960307,
        -0.08726646259971647,
        -0.05235987755982989,
        -0.017453292519943295,
        0.017453292519943295,
        0.05235987755982989,
        0.08726646259971647,
        0.12217304763960307,
        0.15707963267948966,
        0.19198621771937624,
        0.22689280275926285,
        0.2617993877991494
};

class lidar_cluster
{
public:
    lidar_cluster(ros::NodeHandle node,
                  ros::NodeHandle private_nh);
    ~lidar_cluster(){}
    //入口函数
    void pushFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);

    //回调函数---------
    void terrianCallback(const lidar_msg::terrian& msg);
    void genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);

    Point2i trans(Point2f pt);
    //去掉一定高度以外的点
    void clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                   float in_min_height=-1.3, float in_max_height=0.5);
    //取连通域
    void icvprCcaBySeedFill(const Mat& _binImg, Mat& _lableImg);
    //生成clusters
    void genClusters(const Mat label, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, lidar_msg::clusterArrayPtr clusters);
    void colors(const lidar_msg::clusterArrayPtr clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_color_cloud_ptr);
    //ROI
    int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);
    //--------------一些发布函数
    void publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);
    void publishColorCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);
    void publishImage(const ros::Publisher* in_publisher, const Mat inMat);
    void publishclusterArray(const ros::Publisher* in_publisher, const lidar_msg::clusterArrayPtr in_clusterArray);

    std_msgs::Header _velodyne_header;
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_colors;
    ros::Publisher pub_clusters;
    ros::Publisher pub_grid;

    vector<cv::Scalar> _colors;
    int colorsNum;

    //grid
    double gWidth,gHeight;
    double miniGrid;
    int gridW,gridH;
    Mat grid;

    double gridMiH;
    double gridMaH;
    int dilation_size;

    float in_clip_min_height;
    float in_clip_max_height;
    //--------------
    vector<Point2f> ROI;
    pcl::PointCloud<pcl::PointXYZI> ignorPCD;

};

#endif //LIDAR_CLUSTER_LIDARCLUSTER_H
