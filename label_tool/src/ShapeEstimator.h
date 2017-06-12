#ifndef SHAPEESTIMATOR_H
#define SHAPEESTIMATOR_H
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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cv_bridge/cv_bridge.h"//CvBridge中的API可以将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include "cv.h"
#include<sensor_msgs/image_encodings.h> //ROS下的图像的类型，这个头文件中包含对图像进行编码的函数

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "lidar_terrian/GroundResult.h"
#include "lidar_terrian/SegResultDetect.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "Eigen/Eigenvalues"
#include "Eigen/Dense"

typedef struct
{
    float dis_;
    int col;
    pcl::PointXYZL p_;
}point_d;

typedef struct{
    std::vector<float> line;
    int num_p;
}line_;

class ShapeEstimator
{
public:
    ShapeEstimator(pcl::PointCloud<pcl::PointXYZL>  labeled_pc, int label_num);
    ShapeEstimator(pcl::PointCloud<pcl::PointXYZL>  labeled_pc);

    std::vector<std::vector<cv::Point2f> > seg_point;
    std::vector<std::vector<point_d> > nearestpoint;//contours virtual scan
    std::vector<std::vector<std::vector<cv::Point2f> >  >  line_seg;
    std::vector<std::vector<cv::Point2f> > ob;
    std::vector<float > max_z;
    std::vector<float> min_z;
    std::vector<cv::Point3f> max_axis;//not use
    std::vector<cv::Point3f> min_axis;//not use

    std::vector<cv::Point2f> maindirect;

    std::vector<std::vector<cv::Point2f> > four_p;

    std::vector<cv::Point2f> nearsetPoint_P;
    std::vector<cv::Point2f> convertedCVPoint_P;
    cv::Point2f mainDirect_P;
    std::vector<cv::Point2f> box_P;
    float max_z_P;
    float min_z_P;

    int test;
    int halfcloud;
    int seg_num;
    float DISTHREHOLD;
    void nearPoint(pcl::PointCloud<pcl::PointXYZL> labeled_pc, int label_num);
    float distance(pcl::PointXYZL p);
    void testnearestpoint(visualization_msgs::MarkerArray& mk_msg);
    void rerange();
    void IEPF();
    void IEPF_iteration();
    void findBox();
    void findmaindirect();
    void findBoxRANSAC();
    void covfindmaindirect();
    cv::Point2f findMainDirect_RANSAC_P(std::vector<cv::Point2f> in);
    std::vector<cv::Point2f> findBox_P(std::vector<cv::Point2f> in, cv::Point2f direct);
    std::vector<cv::Point2f> convert2CVPoint_P(pcl::PointCloud<pcl::PointXYZL>::Ptr in);
    std::vector<cv::Point2f> findNearest_P(pcl::PointCloud<pcl::PointXYZL>::Ptr in);
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr > pointCloudSeparate(pcl::PointCloud<pcl::PointXYZL>  labeled_pc, int label_num);
    cv::Point2f findLimit_Z(pcl::PointCloud<pcl::PointXYZL>::Ptr in);//x = max  y = min

    cv::Point2f trans(cv::Point2f p, float theta);
    cv::Point2f retrans(cv::Point2f p, float theta);
};








#endif
