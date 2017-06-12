//
// Created by qq77 on 17-4-18.
// this is for classification of 3d LIDAR data with xgboost.
//

#ifndef CLASSIFICATIONTEST_CLASSIFYWITH_H
#define CLASSIFICATIONTEST_CLASSIFYWITH_H

#include<cstdlib>
#include <fstream>
#include <math.h>
#include <iostream>
#include <algorithm>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/ml/ml.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>

#ifndef  _DEBUG
#define  _DEBUG
#endif
using std::cin;
using std::cout;
typedef std::vector<float> Vecf;
typedef std::vector<int> Veci;
typedef pcl::PointCloud<pcl::PointXYZI> Cloudxyzi;

namespace Robosense{

template <class T>
int getArrayLen(T& array)
{//使用模板定义一 个函数getArrayLen,该函数将返回数组array的长度
  return (sizeof(array) / sizeof(array[0]));
}


using namespace std;
using namespace cv;

struct Feature_{
  int Pointnum;
  float Inertia_tensor[6];
  float Cov_mat[3];
  float Cov_eigenvalue[6];
  float Contour_vert[10];
  float Width;
  float Length;
  float Height;
  float distance;
  float Azimuth;
  float Intesity[25];
};

class classifyWithXgboost{

public:

  classifyWithXgboost();
  ~classifyWithXgboost();
  void featureExtraction(const pcl::PointCloud<pcl::PointXYZI> &cloud_, Feature_ &features);

  ///使用xgb2cpp对模型权重进行转换预测
  void xgboostClassification(Feature_ &sampleFeatures,vector<float > &result);
  vector<float> get_probabilities(vector<float> &log_probabilities);

  void feature2Vec(Feature_ &sampleFeature,vector<float >  &featuretmp);//将feature结构体转化为vector

  std::vector<float> xgb_classify(std::vector<float> &sample);//xgb分类器

  // void classifier_xgb(const std::vector<Cloudxyzi >&inClouds, Veci &labels,Vecf &probs,bool usingFeature56);//传入分割好的单个点云，输出预测类别及概率

  int feature_number_;
};
}
#endif //CLASSIFICATIONTEST_CLASSIFYWITH_H
