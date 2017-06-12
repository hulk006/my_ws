#include "FeatureManager.h"
using namespace Robosense;

FeatureManager::FeatureManager(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_, int label_)
{
  label = label_;
  //1:
//    calcuPointnum(cloud_);
  //2:
  calcuInertiaTensor(cloud_);
  //3:
  calcuCovMat(cloud_);
  //4:
//    calcuCovEigenvalue(cloud_);
  //5
  calcuLocalPose(cloud_);
//    calcuLocalPoseNew(cloud_);
  //6
//    calcuIntensity(cloud_);
  //7
//    calcuMidCov(cloud_);
  //8
  calcuSliceMat(cloud_);
  //9
//  calcuIntensityStatistic(cloud_);
  //10
}

FeatureManager::FeatureManager(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)
{
//    Pointnum.resize(1,0);
//    Inertia_tensor.resize(6,0);
//    Cov_mat.resize(6,0);
//    Cov_eigenvalue.resize(6,0);
//    Local_pose.resize(6,0);
//    Intensity.resize(25,0);
//    Mid_cov.resize(6,0);
//    Slice_mat.resize(20,0);
//    Intensity_statistic.resize(4,0);
  //1:
//    calcuPointnum(cloud_);
  //2:
  calcuInertiaTensor(cloud_);
  //3:
  calcuCovMat(cloud_);
  //4:
//    calcuCovEigenvalue(cloud_);
  //5
  calcuLocalPose(cloud_);
//    calcuLocalPoseNew(cloud_);
  //6
//    calcuIntensity(cloud_);
  //7
//    calcuMidCov(cloud_);
  //8
  calcuSliceMat(cloud_);
  //9
  calcuIntensityStatistic(cloud_);
  //10

}

bool FeatureManager::outFileSVMSamples(std::string filename, std::vector<int> choosed_feature)
{
  std::vector<double> feature;
  chooseFeature(choosed_feature,feature);

  std::ofstream outfile;
  outfile.open(filename.c_str(),std::ios::app);

  outfile<<label<<" ";
  int k  = 1;
  for(int i = 0; i < feature.size(); ++i){
    outfile<<k<<":"<<feature[i]<<" ";
    k++;
  }
  outfile<<std::endl;
  outfile.close();
}

void FeatureManager::chooseFeature(std::vector<int> choosedfeature,std::vector<double>& feature)
{
  if(choosedfeature.size() == 0)
    return;
  if(choosedfeature[0] == -1){
//        feature.insert(feature.end(),Pointnum.begin(),Pointnum.end());
    feature.insert(feature.end(),Local_pose.begin(),Local_pose.end());
    feature.insert(feature.end(),Slice_mat.begin(),Slice_mat.end());
    feature.insert(feature.end(),Inertia_tensor.begin(),Inertia_tensor.end());
    feature.insert(feature.end(),Cov_mat.begin(),Cov_mat.end());
    feature.insert(feature.end(),Intensity_statistic.begin(),Intensity_statistic.end());
    return;
  }else{
    for(int i = 0; i < choosedfeature.size();++i){
      if(choosedfeature[i] == 1)
        feature.insert(feature.end(),Pointnum.begin(),Pointnum.end());
      else if(choosedfeature[i] == 2)
        feature.insert(feature.end(),Inertia_tensor.begin(),Inertia_tensor.end());
      else if(choosedfeature[i] == 3)
        feature.insert(feature.end(),Cov_mat.begin(),Cov_mat.end());
      else if(choosedfeature[i] == 4)
        feature.insert(feature.end(),Cov_eigenvalue.begin(),Cov_eigenvalue.end());
      else if(choosedfeature[i] == 5)
        feature.insert(feature.end(),Local_pose.begin(),Local_pose.end());
      else if(choosedfeature[i] == 6)
        feature.insert(feature.end(),Intensity.begin(),Intensity.end());
      else if(choosedfeature[i] == 7)
        feature.insert(feature.end(),Mid_cov.begin(),Mid_cov.end());
      else if(choosedfeature[i] == 8)
        feature.insert(feature.end(),Slice_mat.begin(),Slice_mat.end());
      else if(choosedfeature[i] == 9)
        feature.insert(feature.end(),Intensity_statistic.begin(),Intensity_statistic.end());
    }//end for
    return;
  }
  return;
}

bool FeatureManager::calcuPointnum(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//1
{
  Pointnum.resize(1,0);
  Pointnum[0] = cloud_->points.size();
  return true;
}

bool FeatureManager::calcuInertiaTensor(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//2
{
  Inertia_tensor.resize(6,0);
  for(int pp = 0; pp < cloud_->points.size();++pp){
    Inertia_tensor[0] += (cloud_->points[pp].x) * (cloud_->points[pp].x) + (cloud_->points[pp].y)*(cloud_->points[pp].y);
    Inertia_tensor[1] -= (cloud_->points[pp].x)*(cloud_->points[pp].y);
    Inertia_tensor[2] -= (cloud_->points[pp].x)*(cloud_->points[pp].z);
    Inertia_tensor[3] += (cloud_->points[pp].x) * (cloud_->points[pp].x) + (cloud_->points[pp].z)*(cloud_->points[pp].z);
    Inertia_tensor[4] -= (cloud_->points[pp].y)*(cloud_->points[pp].z);
    Inertia_tensor[5] += (cloud_->points[pp].y) * (cloud_->points[pp].y) + (cloud_->points[pp].z)*(cloud_->points[pp].z);
  }
  return true;
}

bool FeatureManager::calcuCovMat(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//3
{
  Cov_mat.resize(6,0);
  float x_avr = 0;
  float y_avr = 0;
  float z_avr = 0;
  for(int pp = 0; pp < cloud_->points.size();++pp){
    x_avr += cloud_->points[pp].x;
    y_avr += cloud_->points[pp].y;
    z_avr += cloud_->points[pp].z;
  }
  x_avr /= cloud_->points.size();
  y_avr /= cloud_->points.size();
  z_avr /= cloud_->points.size();
  for(int pp = 0; pp < cloud_->points.size();++pp){
    Cov_mat[0] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].x - x_avr);//cov(x,x)
    Cov_mat[1] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].y - y_avr);//cov(x,y)
    Cov_mat[2] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].z - z_avr);//cov(x,z)
    Cov_mat[3] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].y - y_avr);//cov(y,y)
    Cov_mat[4] += (cloud_->points[pp].y - z_avr) * (cloud_->points[pp].y - z_avr);//cov(y,z)
    Cov_mat[5] += (cloud_->points[pp].z - z_avr) * (cloud_->points[pp].z - z_avr);//cov(z,z)
  }
  for(int i = 0; i < 6; ++i){
    Cov_mat[i] /= (cloud_->points.size()-1);
  }
  return true;
}

bool FeatureManager::calcuCovEigenvalue(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//4
{
  Eigen::Matrix3f C;
  C(0,0) = Cov_mat[0];    C(1, 1) = Cov_mat[3];   C(2,2) = Cov_mat[5];
  C(0,1) = C(1,0) = Cov_mat[1];
  C(0,2) = C(2,0) = Cov_mat[2];
  C(1,2) = C(2,1) = Cov_mat[4];
  Eigen::EigenSolver<Eigen::Matrix3f> es(C);
  Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
  Cov_eigenvalue.resize(6,0);
  float eigenvalue_sum = 0;
  for(int aa = 0; aa < 3; ++aa){
    if(fabs(D(aa,aa)) < 0.0001){
      Cov_eigenvalue[aa] = 0;
    }
    else{Cov_eigenvalue[aa] = D(aa,aa);}
    eigenvalue_sum += Cov_eigenvalue[aa];
  }
  for(int nn = 0 ; nn < 3 ; ++nn){
    for(int mm = 0; mm < (3-nn-1) ; mm++){
      if(Cov_eigenvalue[mm] < Cov_eigenvalue[mm+1]){
        float temp_ = Cov_eigenvalue[mm];
        Cov_eigenvalue[mm] = Cov_eigenvalue[mm+1];
        Cov_eigenvalue[mm + 1] = temp_;
      }
    }
  }
  if(eigenvalue_sum != 0){
    for(int nn = 0; nn <3 ; nn++){
      Cov_eigenvalue[3+nn] = Cov_eigenvalue[nn] / eigenvalue_sum;
    }
  }
  return true;
}

bool FeatureManager::calcuLocalPose(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//5
{
  Local_pose.resize(6,0);
  ShapeEstimator se(cloud_);
  Local_pose[2] = se.max_z_P - se.min_z_P;//Height
  float dis1 = (se.box_P[0].x - se.box_P[1].x)*(se.box_P[0].x - se.box_P[1].x)+(se.box_P[0].y - se.box_P[1].y)*(se.box_P[0].y - se.box_P[1].y);
  dis1 = sqrt(dis1);
  float dis2 = (se.box_P[2].x - se.box_P[1].x)*(se.box_P[2].x - se.box_P[1].x)+(se.box_P[2].y - se.box_P[1].y)*(se.box_P[2].y - se.box_P[1].y);
  dis2 = sqrt(dis2);
  Local_pose[1] = dis1 > dis2 ? dis1 : dis2;//Length
  Local_pose[0] = dis1 < dis2 ? dis1 : dis2;//Width
  std::vector<float> box_center(3,0);//x,y,z
  for(int pp = 0; pp < 4 ; ++pp){
    box_center[0] += se.box_P[0].x;
    box_center[1] += se.box_P[0].y;
  }
  box_center[0] /= 4;   box_center[1] /= 4;   box_center[2] = se.min_z_P + Local_pose[2]/2;
  Local_pose[3] =  sqrt( (box_center[0]) * (box_center[0]) + (box_center[1]) * (box_center[1]) + (box_center[2]) * (box_center[2])   );
  //Distance
  Local_pose[4] = atan2(box_center[1], box_center[0] );
  //Azimuth
  Local_pose[5] = atan2(fabs(se.mainDirect_P.y), fabs(se.mainDirect_P.x));
  //Orientation
}

bool FeatureManager::calcuLocalPoseNew(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//5
{
  Local_pose.resize(6,0);
  pcl::PointXYZI max,min;
  pcl::getMinMax3D(*cloud_,min,max);
  float deta_x = max.x - min.x;
  float deta_y = max.y - min.y;
  Local_pose[2] = max.z - min.z;//Height

  vector<cv::Point2f >    target_points ;
  target_points.clear();
  for (int i = 0;i< cloud_->points.size();i++)
  {
    cv::Point2f   target_point;
    target_point.x = cloud_->points[i].x - min.x;
    target_point.y = cloud_->points[i].y - min.y;
    target_points.push_back( target_point);
  }
  BoxEstimater  rect;
//    rect_info target_rectangle;
  rect.Rotated_point(target_points , target_rectangle, (int)10*deta_y+1, (int)10*deta_x+1);

  Local_pose[1] = target_rectangle.length > target_rectangle.width ? target_rectangle.length : target_rectangle.width;//Length
  Local_pose[0] = target_rectangle.length < target_rectangle.width ? target_rectangle.length : target_rectangle.width;//Width

  Local_pose[3] =  sqrt( (target_rectangle.center.x) * (target_rectangle.center.x) + (target_rectangle.center.y) * (target_rectangle.center.x) + (max.z - min.z) * (max.z - min.z)/4   );
  //Distance
  Local_pose[4] = atan2(target_rectangle.center.x, target_rectangle.center.y);
  //Azimuth
  Local_pose[5] = target_rectangle.main_derection;
  //Orientation
}

bool FeatureManager::calcuIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//6
{
  Intensity.resize(25,0);
  for(int pp = 0; pp < cloud_->points.size();++pp){
    int index = cloud_ -> points[pp].intensity/25;
    if((index >= 0) && (index < 25 ))
      Intensity[index] += 1;
  }
  for(int i = 0; i < 25 ; ++i){Intensity[i] /= cloud_->points.size();}
}

bool FeatureManager::calcuMidCov(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//8
{
  Mid_cov.resize(6,0);
  pcl::PointXYZI pointMid;
  std::vector<float> xcord;
  std::vector<float> ycord;
  std::vector<float> zcord;
  for (int i = 0; i < cloud_->points.size();i++){
    xcord.push_back(cloud_->points[i].x);
    ycord.push_back(cloud_->points[i].y);
    zcord.push_back(cloud_->points[i].z);
  }
  //sort the cord based xcord
  int n = xcord.size();
  int k = 0, j = 0;
  float t = 0;
  for (int i = 0; i < n - 1; i++){
    k = i;
    for (j = i + 1; j < n; j++){
      if (xcord[j] < xcord[k])
        k = j;
    }
    if (k != i){
      t = xcord[i];
      xcord[i] = xcord[k];
      xcord[k] = t;
    }
  }
  //sort the cord based ycord
  k = 0;
  j = 0;
  t = 0;
  for (int i = 0; i < n - 1; i++){
    k = i;
    for (j = i + 1; j < n; j++){
      if (ycord[j] < ycord[k])
        k = j;
    }
    if (k != i){
      t = ycord[i];
      ycord[i] = ycord[k];
      ycord[k] = t;
    }
  }

  //sort the cord based zcord
  k = 0;
  j = 0;
  t = 0;
  for (int i = 0; i < n - 1; i++){
    k = i;
    for (j = i + 1; j < n; j++){
      if (zcord[j] < zcord[k])
        k = j;
    }
    if (k != i){
      t = zcord[i];
      zcord[i] = zcord[k];
      zcord[k] = t;
    }
  }
  //return the middle value
  pointMid.x = xcord[n / 2];
  pointMid.y = ycord[n / 2];
  pointMid.z = zcord[n / 2];
  for (int i = 0; i < cloud_->points.size(); i++){
    Mid_cov[0] += ((cloud_->points[i].x) - pointMid.x) * ((cloud_->points[i].y) - pointMid.y);//xy
    Mid_cov[1] += ((cloud_->points[i].x) - pointMid.x) * ((cloud_->points[i].z) - pointMid.z);//xz
    Mid_cov[2] += ((cloud_->points[i].y) - pointMid.y) * ((cloud_->points[i].z) - pointMid.z);//yz
    Mid_cov[3] += ((cloud_->points[i].x) - pointMid.x) * ((cloud_->points[i].x) - pointMid.x);//xx
    Mid_cov[4] += ((cloud_->points[i].y) - pointMid.y) * ((cloud_->points[i].y) - pointMid.y);//yy
    Mid_cov[5] += ((cloud_->points[i].z) - pointMid.z) * ((cloud_->points[i].z) - pointMid.z);//zz
  }
  for (int aa = 0; aa < 6; ++aa){
    Mid_cov[aa] /= (cloud_->points.size() - 1);
  }

  return true;
}

bool FeatureManager::calcuSliceMat(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//9
{
  float min_z = 100;
  float max_z = -100;
  for (int i = 0; i < cloud_->points.size(); i++)
  {
    if (cloud_->points[i].z < min_z)
      min_z = cloud_->points[i].z;
    if (cloud_->points[i].z > max_z)
      max_z = cloud_->points[i].z;
  }
  int sliceNum = 10;
  Slice_mat.resize(sliceNum*2,0);
  float sliceStep = (max_z - min_z) / sliceNum;
//    std::cout<<sliceStep<<std::endl;
  if(sliceStep <0.01)
    return false;
  std::vector<std::vector<pcl::PointXYZI> > sliceHistgram;
  sliceHistgram.resize(sliceNum);
  for (int i = 0; i < cloud_->points.size(); i++)
  {
    int sliceIndex = (cloud_->points[i].z - min_z) / sliceStep;
    if(sliceIndex == sliceNum)
      sliceIndex -= 1;
    sliceHistgram[sliceIndex].push_back(cloud_->points[i]);
  }

  for (int i = 0; i < sliceHistgram.size(); i++)
  {
    float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    if (sliceHistgram[i].size() > 0)
    {
      min_x = sliceHistgram[i][0].x;
      max_x = sliceHistgram[i][0].x;
      min_y = sliceHistgram[i][0].y;
      max_y = sliceHistgram[i][0].y;

    }
    for (int j = 0; j < sliceHistgram[i].size(); j++)
    {
      if (sliceHistgram[i][j].x < min_x)
        min_x = sliceHistgram[i][j].x;
      if (sliceHistgram[i][j].x > max_x)
        max_x = sliceHistgram[i][j].x;

      if (sliceHistgram[i][j].y < min_y)
        min_y = sliceHistgram[i][j].y;
      if (sliceHistgram[i][j].y > max_y)
        max_y = sliceHistgram[i][j].y;
    }
    float l = 0, w = 0;
    if ((max_x - min_x) > (max_y - min_y))
    {
      l = max_x - min_x;
      w = max_y - min_y;
    }
    else
    {
      w = max_x - min_x;
      l = max_y - min_y;
    }

    Slice_mat[2*i] = l;
    Slice_mat[2*i + 1] = w;
  }
  return true;
}

bool FeatureManager::calcuIntensityStatistic(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_)//10
{
  Intensity_statistic.resize(4,0);
  float intensityMax = 0,intensityMin = 255,intensityMean = 0,intensityDev = 0;
  for(int i = 0; i<cloud_->points.size();i++)
  {
    intensityMean += cloud_->points[i].intensity;
    intensityMax = cloud_->points[i].intensity > intensityMax ? cloud_->points[i].intensity : intensityMax;
    intensityMin = cloud_->points[i].intensity < intensityMin ? cloud_->points[i].intensity : intensityMin;
  }
  intensityMean /= cloud_->points.size();
  for(int i = 0; i<cloud_->points.size();i++)
  {
    intensityDev += (cloud_->points[i].intensity - intensityMean)*(cloud_->points[i].intensity - intensityMean);
  }
  intensityDev = sqrt(intensityDev);
  intensityDev /= cloud_->points.size();
  Intensity_statistic[0] = intensityMax;
  Intensity_statistic[1] = intensityMin;
  Intensity_statistic[2] = intensityMean;
  Intensity_statistic[3] = intensityDev;
}
