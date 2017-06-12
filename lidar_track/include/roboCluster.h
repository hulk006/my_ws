//
// Created by mjj on 17-5-26.
//

#ifndef PROJECT_ROBOCLUSTER_H
#define PROJECT_ROBOCLUSTER_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
#include <opencv2/opencv.hpp>
#include <iostream>

#define BEAMSNUM_16 16

namespace Robosense{
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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;

    class roboCluster{
    public:
        roboCluster();
        ~roboCluster(){};
        void roboClusterInit(std::string str,double maxHeight = 50.,double maxWidth = 50.,double miniGrid = 0.15,
                             int dilaSize = 1,double clip_min_height = -1.8,double clip_max_height = 0.3);

        void cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud_ptr,std::vector<pcl::PointCloud<pcl::PointXYZI> >& out_clusters);

        void tranSort(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr);

    private:
        void preProcFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr);
        void computeFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_ori_cloud_ptr,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pre_cloud_ptr,
                            pcl::PointCloud<MyPointType>::Ptr out_feature_ptr);
        void genFeatureMat(const pcl::PointCloud<MyPointType>::Ptr in_feature_ptr,cv::Mat& out_feature_mat);
        void genTerrian(const cv::Mat in_feature_mat,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud_ptr);
        float comPuteDis3D(cv::Point3f pt1, cv::Point3f pt2);
        bool ignore(cv::Point3f pt);
        cv::Point2i computeIndex(cv::Point2f pt);

        void genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                     const int gridH,const int gridW,cv::Mat& grid);
        cv::Point2i trans(cv::Point2f pt);
        //去掉一定高度以外的点
        void clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                       float in_min_height=-1.3, float in_max_height=0.5);
        //取连通域
        void icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg);
        //生成clusters
        void genClusters(const cv::Mat label, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                         std::vector<pcl::PointCloud<pcl::PointXYZI> >& out_clusters);


        CvDTree* dtree;
        double maxH,maxW;
        double BiasX,BiasY;
        float stepGrid;
        cv::Mat gridLowestH;

        double gWidth,gHeight,miniG;
        int gridW,gridH;
        cv::Mat grid;
        int dilation_size;
        double in_clip_min_height,in_clip_max_height;

        bool initFlag;
        pcl::PointCloud<pcl::PointXYZI> ignorePts;

    };
}

#endif //PROJECT_ROBOCLUSTER_H
