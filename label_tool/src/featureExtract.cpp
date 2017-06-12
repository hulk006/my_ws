#include "featureExtract.h"
FeatureExtract::FeatureExtract()
{
    PointNum = 0;
    Inertia_tensor[6] = {0};//
    Cov_mat[6] = {0};//
    Cov_eigenvalue[6] = {0};//
    Contour_vert[10] = {0};
    Intensity[25] = {0};
    Width = 0;//
    Length = 0;//
    Height = 0;//
    Distance = 0;//
    Azimuth = 0;//
}
FeatureExtract::~FeatureExtract()
{


}

std::vector<float> FeatureExtract::GetFpfhDescriptor(pcl::PointCloud<pointType>::Ptr pointcloud_, const unsigned int& in_ompnum_threads, const double& in_normal_search_radius, const double& in_fpfh_search_radius)
{
    std::vector<float> cluster_fpfh_histogram(33, 0.0);

    pcl::search::KdTree<pointType>::Ptr norm_tree(new pcl::search::KdTree<pointType>);
    if (pointcloud_->points.size() > 0)
        norm_tree->setInputCloud(pointcloud_);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pointType, pcl::Normal> normal_estimation;
    normal_estimation.setNumberOfThreads(in_ompnum_threads);
    normal_estimation.setInputCloud(pointcloud_);
    normal_estimation.setSearchMethod(norm_tree);
    normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    normal_estimation.setRadiusSearch(in_normal_search_radius);
    normal_estimation.compute(*normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_histograms(new pcl::PointCloud<pcl::FPFHSignature33>());

    pcl::FPFHEstimationOMP<pointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(in_ompnum_threads);
    fpfh.setInputCloud(pointcloud_);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(norm_tree);
    fpfh.setRadiusSearch(in_fpfh_search_radius);
    fpfh.compute(*fpfh_histograms);

    float fpfh_max = std::numeric_limits<float>::min();
    float fpfh_min = std::numeric_limits<float>::max();

    for (unsigned int i = 0; i < fpfh_histograms->size(); i++) //for each point fpfh
    {
        for (unsigned int j = 0; j < cluster_fpfh_histogram.size(); j++)//sum each histogram's bin for all points, get min/max
        {
            cluster_fpfh_histogram[j] = cluster_fpfh_histogram[j] + fpfh_histograms->points[i].histogram[j];
            if (cluster_fpfh_histogram[j] < fpfh_min)
                fpfh_min = cluster_fpfh_histogram[j];
            if (cluster_fpfh_histogram[j] > fpfh_max)
                fpfh_max = cluster_fpfh_histogram[j];
        }

        float fpfh_dif = fpfh_max - fpfh_min;
        for (unsigned int j = 0; fpfh_dif > 0, j < cluster_fpfh_histogram.size(); j++)//substract the min from each and normalize
        {
            cluster_fpfh_histogram[j] = (cluster_fpfh_histogram[j] - fpfh_min) / fpfh_dif;
        }
    }

    return cluster_fpfh_histogram;
}

pointType FeatureExtract::midValue(pcl::PointCloud<pointType>::Ptr pointcloud_)
{
    std::vector<float> xcord;
    std::vector<float> ycord;
    std::vector<float> zcord;
    for (int i = 0; i < pointcloud_->points.size();i++)
    {
        xcord.push_back(pointcloud_->points[i].x);
        ycord.push_back(pointcloud_->points[i].y);
        zcord.push_back(pointcloud_->points[i].z);
    }
    //sort the cord based xcord
    pointType point;
    int n = xcord.size();
    int k = 0, j = 0;
    float t = 0;
    for (int i = 0; i < n - 1; i++)
    {
        k = i;
        for (j = i + 1; j < n; j++)
        {
            if (xcord[j] < xcord[k])
                k = j;
        }
        if (k != i)
        {
            t = xcord[i];
            xcord[i] = xcord[k];
            xcord[k] = t;

        }

    }
    //sort the cord based ycord
    k = 0;
    j = 0;
    t = 0;
    for (int i = 0; i < n - 1; i++)
    {
        k = i;
        for (j = i + 1; j < n; j++)
        {
            if (ycord[j] < ycord[k])
                k = j;
        }
        if (k != i)
        {
            t = ycord[i];
            ycord[i] = ycord[k];
            ycord[k] = t;

        }

    }

    //sort the cord based zcord
    k = 0;
    j = 0;
    t = 0;
    for (int i = 0; i < n - 1; i++)
    {
        k = i;
        for (j = i + 1; j < n; j++)
        {
            if (zcord[j] < zcord[k])
                k = j;
        }
        if (k != i)
        {
            t = zcord[i];
            zcord[i] = zcord[k];
            zcord[k] = t;

        }

    }
    //return the middle value
    point.x = xcord[n / 2];
    point.y = ycord[n / 2];
    point.z = zcord[n / 2];

    return point;
}

std::vector<float> FeatureExtract::GetMeanDevFromMed(pcl::PointCloud<pointType>::Ptr pointcloud_)
{
    std::vector<float> Cov_mid_mat(6);
    pointType pointMid = midValue(pointcloud_);
    for (int i = 0; i < pointcloud_->points.size(); i++)
    {
        Cov_mid_mat[0] += ((pointcloud_->points[i].x) - pointMid.x) * ((pointcloud_->points[i].y) - pointMid.y);//xy
        Cov_mid_mat[1] += ((pointcloud_->points[i].x) - pointMid.x) * ((pointcloud_->points[i].z) - pointMid.z);//xz
        Cov_mid_mat[2] += ((pointcloud_->points[i].y) - pointMid.y) * ((pointcloud_->points[i].z) - pointMid.z);//yz
        Cov_mid_mat[3] += ((pointcloud_->points[i].x) - pointMid.x) * ((pointcloud_->points[i].x) - pointMid.x);//xx
        Cov_mid_mat[4] += ((pointcloud_->points[i].y) - pointMid.y) * ((pointcloud_->points[i].y) - pointMid.y);//yy
        Cov_mid_mat[5] += ((pointcloud_->points[i].z) - pointMid.z) * ((pointcloud_->points[i].z) - pointMid.z);//zz
    }
    for (int aa = 0; aa < 6; ++aa)
    {
        Cov_mid_mat[aa] /= (pointcloud_->points.size() - 1);
    }
    return Cov_mid_mat;
}

std::vector<float> FeatureExtract::GetSliceDescriptor(pcl::PointCloud<pointType>::Ptr pointcloud_)
{
    std::vector<float> slice_mat;
    float min_z = 100;
    float max_z = -100;
    for (int i = 0; i < pointcloud_->points.size(); i++)
    {
        if (pointcloud_->points[i].z < min_z)
            min_z = pointcloud_->points[i].z;
        if (pointcloud_->points[i].z > max_z)
            max_z = pointcloud_->points[i].z;
    }

    int sliceNum = 10;
    slice_mat.resize(sliceNum*2,0);
    float sliceStep = (max_z - min_z) / sliceNum;
//    std::cout<<sliceStep<<std::endl;
    if(sliceStep <0.01)
        return slice_mat;
    std::vector<std::vector<pointType> > sliceHistgram;
    sliceHistgram.resize(sliceNum);
    for (int i = 0; i < pointcloud_->points.size(); i++)
    {
        int sliceIndex = (pointcloud_->points[i].z - min_z) / sliceStep;
        if(sliceIndex == sliceNum)
            sliceIndex -= 1;
        sliceHistgram[sliceIndex].push_back(pointcloud_->points[i]);
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

        slice_mat[2*i] = l;
        slice_mat[2*i + 1] = w;
    }
    return slice_mat;
}

std::vector<float> FeatureExtract::GetIntensityStatistic(pcl::PointCloud<pointType>::Ptr pointcloud_)
{
    std::vector<float> Intensity_statis;
    float intensityMax = 0,intensityMin = 255,intensityMean = 0,intensityDev = 0;
    for(int i = 0; i<pointcloud_->points.size();i++)
    {
        intensityMean += pointcloud_->points[i].intensity;
        intensityMax = pointcloud_->points[i].intensity > intensityMax ? pointcloud_->points[i].intensity : intensityMax;
        intensityMin = pointcloud_->points[i].intensity < intensityMin ? pointcloud_->points[i].intensity : intensityMin;
    }
    intensityMean /= pointcloud_->points.size();
    for(int i = 0; i<pointcloud_->points.size();i++)
    {
        intensityDev += (pointcloud_->points[i].intensity - intensityMean)*(pointcloud_->points[i].intensity - intensityMean);
    }
    intensityDev = sqrt(intensityDev);
    intensityDev /= pointcloud_->points.size();
    Intensity_statis.push_back(intensityMax);
    Intensity_statis.push_back(intensityMin);
    Intensity_statis.push_back(intensityMean);
    Intensity_statis.push_back(intensityDev);
    return Intensity_statis;
}

void FeatureExtract::featureExtract(pcl::PointCloud<pointType>::Ptr pointcloud_, int label)
{
    fpfh = GetFpfhDescriptor(pointcloud_,1,0.3,0.3);
//                     std::cout<<"2"<<std::endl;
    slice = GetSliceDescriptor(pointcloud_);
//                     std::cout<<"3"<<std::endl;
    meanDevMat = GetMeanDevFromMed(pointcloud_);
//                     std::cout<<"4"<<std::endl;
   Intensity_statis = GetIntensityStatistic(pointcloud_);
//                    std::cout<<"5"<<std::endl;

    std::ofstream outfile;
        outfile.open("/home/xcon/data/f7fphf.txt", std::ios::app);
     outfile<< label << " ";//
     for(int i = 0; i<fpfh.size();i++)
     {
         outfile<<i+1<< ":"<<fpfh[i]<<" ";
     }
     outfile<<std::endl;
     outfile.close();

     outfile.open("/home/xcon/data/f8slice.txt", std::ios::app);
     outfile<< label << " ";//
     for(int i = 0; i<slice.size();i++)
     {
         outfile<<i+1<< ":"<<slice[i]<<" ";
     }
     outfile<<std::endl;
     outfile.close();

     outfile.open("/home/xcon/data/f9slice.txt", std::ios::app);
     outfile<< label << " ";//
     for(int i = 0; i<meanDevMat.size();i++)
     {
         outfile<<i+1<< ":"<<meanDevMat[i]<<" ";
     }
     outfile<<std::endl;
     outfile.close();

     outfile.open("/home/xcon/data/f10slice.txt", std::ios::app);
     outfile<< label << " ";//
     for(int i = 0; i<Intensity_statis.size();i++)
     {
         outfile<<i+1<< ":"<<Intensity_statis[i]<<" ";
     }
     outfile<<std::endl;
     outfile.close();

}


