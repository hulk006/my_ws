//
// Created by mjj on 17-5-2.
//

#ifndef LIDAR_CLUSTER_LIDARTERRIAN_H
#define LIDAR_CLUSTER_LIDARTERRIAN_H

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
#include <ros/package.h>
#include <dirent.h>
#include "lidar_msg/terrian.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
#define BEAMS 16

struct weakFeature
{
    float f1_Range;
    float f2_Intensity;
    float f3_Lrange;
    float f4_Rrange;
    float f5_Trange;
    float f6_Brange;
    float f7_DisBelow;
    float f8_HeightInLocal;
};


struct MyPointType
{
    PCL_ADD_POINT4D;
    weakFeature wF;
    bool tagF;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

class terrian_estimate
{
public:
    terrian_estimate(ros::NodeHandle node,
                     ros::NodeHandle private_nh);

    //回调函数
    void terrianCallback(const sensor_msgs::PointCloud2& msg);

    void pushFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);
    void preProcFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr);
    void computeFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_ori_cloud_ptr,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pre_cloud_ptr,
                        pcl::PointCloud<MyPointType>::Ptr out_feature_ptr);
    void genFeatureMat(const pcl::PointCloud<MyPointType>::Ptr in_feature_ptr,Mat& out_feature_mat);
    void genTerrian(const Mat in_feature_mat,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                    lidar_msg::terrianPtr out_terrian);
    void genFloor(const lidar_msg::terrianPtr in_terrian,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr floor_cloud_ptr,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud_ptr);

    ~terrian_estimate(){}
    //--------------publish--------------------------------
    void publishCloud(const ros::Publisher* in_publisher,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);
    void publishTerrian(const ros::Publisher* in_publisher,
                        const lidar_msg::terrianPtr in_terrian);
    //------------------------------------------------------

    Point2i computeIndex(Point2f pt);//
    bool ignore(Point3f pt);
    float comPuteDis3D(Point3f pt1, Point3f pt2);

    Mat gridLowestH;
    CvDTree* dtree;

    //--------------pub-----------
    ros::Subscriber sub_pcd;
    ros::Publisher pub_floor;
    ros::Publisher pub_nofloor;
    ros::Publisher pub_terrian;
    std_msgs::Header _velodyne_header;
    //--------------terrian----------
    int maxH,maxW;
    int BiasX, BiasY;//
    float stepGrid;//
};

#endif //LIDAR_CLUSTER_LIDARTERRIAN_H
