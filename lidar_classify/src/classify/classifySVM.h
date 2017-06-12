#ifndef CLASSIFYSVM_H
#define CLASSIFYSVM_H

#include<ros/ros.h>
#include <ros/package.h>
#include<cstdlib>
#include <fstream>
#include <math.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "classifier.h"
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
#include"FeatureManager.h"
//typedef Eigen::Matrix<double,3,1> mat_12;
namespace Robosense{
typedef struct
{
    double max;
    double min;
}Feature_Range;


class ClassifierSVM
{
public:
    ClassifierSVM(const std::string& modelfile,  const std::string& rangefile);
    ClassifierSVM();
    ~ClassifierSVM(){}
    void setModelRange(const std::string& modelfile,  const std::string& rangefile);
    int objectClassify(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, vector<float> &prob);
    int objectClassify(std::vector<double>& feature, vector<float> &prob);
    struct svm_model *model_;

private:
    void loadRange(const std::string& rangefilename, std::vector<Feature_Range> &range);
    void rerangeFeature(std::vector<double> &feature);
    void setSelectfeature(std::vector<int> sf);


    std::vector<Feature_Range> feature_range_;
    double MAX_R_;
    double MIN_R_;
    std::vector<int> selectfeature_;
};

class MultiClassifierSVM
{
public:
    MultiClassifierSVM(std::vector<std::string> modelRangeFile);
    MultiClassifierSVM();
    ~MultiClassifierSVM(){}
    std::vector<float> MultiClassify(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    bool setModelRange(std::vector<std::string> modelRangeFile);
    void setSelectfeature(std::vector<int> sf);

    std::vector<ClassifierSVM> mcf_;
    int predictLabel_;
    float predictProb_;
    std::vector<int> selectfeature_;
};

}
#endif

