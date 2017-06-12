#ifndef FEATUREEXTRACT_H
#define FEATUREEXTRACT_H

#include<cstdlib>
#include <fstream>
#include <math.h>

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
#include <map>
#include <queue>
//#include "Graph.h"
//#include<image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/ml/ml.hpp>
#include "ShapeEstimator.h"
using namespace std;
using namespace cv;
typedef pcl::PointXYZI pointType;

#define PI 3.1415926
struct BoundBox
{
    float max_x;
    float max_y;
    float max_z;
    float min_x;
    float min_y;
    float min_z;
};

class FeatureExtract
{
public:
    FeatureExtract();
    ~FeatureExtract();
    void featureExtract(pcl::PointCloud<pointType>::Ptr pointcloud_,int label);
    std::vector<float> GetFpfhDescriptor(pcl::PointCloud<pointType>::Ptr pointcloud_,const unsigned int& in_ompnum_threads, const double& in_normal_search_radius, const double& in_fpfh_search_radius);
    std::vector<float> GetMeanDevFromMed(pcl::PointCloud<pointType>::Ptr pointcloud_);
    pointType midValue(pcl::PointCloud<pointType>::Ptr pointcloud_);
    std::vector<float> GetSliceDescriptor(pcl::PointCloud<pointType>::Ptr pointcloud_);
    std::vector<float> GetIntensityStatistic(pcl::PointCloud<pointType>::Ptr pointcloud_);
    int PointNum;//
    float Inertia_tensor[6];//
    float Cov_mat[6];//
    float Cov_eigenvalue[6];//
    float Contour_vert[10];
    float Width;//
    float Length;//
    float Height;//
    float Distance;//
    float Azimuth;//
    float Intensity[25];
    std::vector<float> fpfh;
    std::vector<float> meanDevMat;
    std::vector<float> slice;
    std::vector<float> Intensity_statis;
};
#endif
