#ifndef CLASSIFYTRACK_H
#define CLASSIFYTRACK_H

#include<ros/ros.h>
#include <ros/package.h>
#include<cstdlib>
#include <fstream>
#include <math.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//#include "lidar_terrian/GroundResult.h"
//#include "lidar_terrian/SegResultDetect.h"
#include "lidar_msg/clusterArray.h"
#include <map>
#include <queue>
//#include "Graph.h"

#include "cv_bridge/cv_bridge.h"//CvBridge中的API可以将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include "cv.h"
#include<sensor_msgs/image_encodings.h> //ROS下的图像的类型，这个头文件中包含对图像进行编码的函数
//#include<image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/ml/ml.hpp>


#include "track.h"
#include "classifyWithXgboost.h"

//typedef Eigen::Matrix<double,3,1> mat_12;
using namespace std;
using namespace cv;

#define CAR 1
#define TRUCK 2
#define PED 3
#define BKG 4

/*
struct Feature_
{
    int Pointnum;//
    float Inertia_tensor[6];//
    float Cov_mat[3];//
    float Cov_eigenvalue[6];//
    float Contour_vert[10];
    float Width;//
    float Length;//
    float Height;//
    float distance;//
    float Azimuth;//
    float Intensity[25];

};
 */


struct VNODE
{
    pcl::PointXYZI point;
    int clusterIdx;
    std::vector<int> rNN;
};

struct MaxMin
{
    float max_x;
    float max_y;
    float max_z;
    float min_x;
    float min_y;
    float min_z;
};


class xgbClassifyTrack
{
public:
    xgbClassifyTrack(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~xgbClassifyTrack();
   

private:
    void classifyTrackCallback(const lidar_msg::clusterArray &msg);
    void initClass();
    ros::Subscriber seg_sub;
    ros::Publisher vel_pub;
    ros::Publisher pc_pub;
    Robosense::Tracking MM;
    ros::Publisher marker_pub;
    visualization_msgs::MarkerArray markerarray;
};


#endif

