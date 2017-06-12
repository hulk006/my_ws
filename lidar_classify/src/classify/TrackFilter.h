#ifndef TRACKFILTER_H
#define TRACKFILTER_H
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

#include "Eigen/Eigenvalues"
#include "Eigen/Dense"
using namespace std;
class TrackFilter
{
public:
    TrackFilter(vector<float> vprob);
    float prob;
    void computeFilterProb(vector<float> vprob);
};








#endif
