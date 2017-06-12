
#ifndef FEATUREMANAGER_H
#define FEATUREMANAGER_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
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
#include "ShapeEstimator.h"
#include "boxEstimater.h"

namespace Robosense{
class FeatureManager
{
    typedef std::vector<float> vecf;

public:
    FeatureManager(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_, int label);//for file
    FeatureManager(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);

    vecf Pointnum;//1   1
    std::vector<float> Inertia_tensor;//2   6
    std::vector<float> Cov_mat;//3    6
    std::vector<float> Cov_eigenvalue;//4    6
    std::vector<float> Local_pose;//5     6
    // float Width;    float Length;    float Height;    float Distance;  float Azimuth;   float Orientation;
    //std::vector<float> Contour_vert;
    std::vector<float> Intensity;//6      25

    std::vector<float> Mid_cov;//7      6
    std::vector<float> Slice_mat;//8    20
    std::vector<float> Intensity_statistic;//9     4
    int label;

    //int feature_dimension;
    rect_info target_rectangle;

    bool outFileSVMSamples(std::string filename,std::vector<int> choosed_feature);
    void chooseFeature(std::vector<int> choosedfeature,std::vector<double>& feature);
    bool calcuPointnum(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuInertiaTensor(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuCovMat(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuCovEigenvalue(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuLocalPose(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuLocalPoseNew(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuFPFH(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuMidCov(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuSliceMat(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
    bool calcuIntensityStatistic(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_);
};

}






#endif
