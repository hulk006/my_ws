//
// Created by mjj on 17-5-2.
//
#include "../include/lidarTerrian.h"


terrian_estimate::terrian_estimate(ros::NodeHandle node,
                                   ros::NodeHandle private_nh){
    string topic = private_nh.param("topic",string("rslidar"));
    //string topic = private_nh.param("topic",string("pcd"));
    sub_pcd = node.subscribe(topic,10,&terrian_estimate::terrianCallback,(terrian_estimate*)this);
    pub_floor = node.advertise<sensor_msgs::PointCloud2>("floor",10);
    pub_nofloor = node.advertise<sensor_msgs::PointCloud2>("nofloor",10);
    pub_terrian = node.advertise<lidar_msg::terrian>("terrian",10);

    string pwd = ros::package::getPath("lidar_cluster");
    string model_str = private_nh.param("model",pwd + "/model/dtree.xml");
    stepGrid = private_nh.param("stepGrid",float(2.));
    maxH = private_nh.param("maxH",int(50));
    maxW = private_nh.param("maxW",int(50));

    BiasY = int(maxH / stepGrid) + 1;
    BiasX = int(maxW / stepGrid) + 1;
    gridLowestH = Mat::zeros(BiasY * 2, BiasX * 2, CV_64F);
    dtree = new CvDTree;
    dtree->load(model_str.c_str());
}

void terrian_estimate::terrianCallback(const sensor_msgs::PointCloud2& msg){
    _velodyne_header =msg.header;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*tmp_cloud_ptr);

    if (tmp_cloud_ptr->height != BEAMS){
        int height = BEAMS;
        int width = tmp_cloud_ptr->size() / height;
        cloud->height = height;
        cloud->width = width;
        cloud->resize(width * height);
        int iid = 0;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                cloud->points[j * width + i] = tmp_cloud_ptr->points[iid];
                iid++;
            }
        }
    }else{
        *cloud = *tmp_cloud_ptr;
    }

    pushFrame(cloud);
}

void terrian_estimate::pushFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr){

    gridLowestH.setTo(INT_MAX);
    pcl::PointCloud<pcl::PointXYZI>::Ptr preProd_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<MyPointType>::Ptr feature_cloud_ptr(new pcl::PointCloud<MyPointType>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr floor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    lidar_msg::terrianPtr terrianRes(new lidar_msg::terrian);

    Mat featureMat;

    preProcFrame(in_cloud_ptr,preProd_cloud_ptr);
    computeFeature(in_cloud_ptr,preProd_cloud_ptr,feature_cloud_ptr);
    genFeatureMat(feature_cloud_ptr,featureMat);
    genTerrian(featureMat,in_cloud_ptr,terrianRes);
    genFloor(terrianRes,floor_cloud_ptr,nofloor_cloud_ptr);

    publishCloud(&pub_floor,floor_cloud_ptr);
    publishCloud(&pub_nofloor,nofloor_cloud_ptr);
    publishTerrian(&pub_terrian,terrianRes);
}

void terrian_estimate::preProcFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr){

    int width = in_cloud_ptr->width;
    int height = in_cloud_ptr->height;
    out_cloud_ptr->width = width;
    out_cloud_ptr->height = height;
    out_cloud_ptr->resize(width * height);

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            pcl::PointXYZI tmpPt = in_cloud_ptr->points[j*width + i];
            pcl::PointXYZI tmpFor;

            if (abs(tmpPt.x) > maxW || abs(tmpPt.y) > maxH){
                in_cloud_ptr->points[j*width + i].x = 0;
                in_cloud_ptr->points[j*width + i].y = 0;
                in_cloud_ptr->points[j*width + i].z = 0;
                tmpPt = in_cloud_ptr->points[j*width + i];
            }
            if (!ignore(Point3f(tmpPt.x,tmpPt.y,tmpPt.z))){
                out_cloud_ptr->points[j*width + i] = in_cloud_ptr->points[j*width + i];
                continue; //如果不是空点，则跳过
            }

            int idx_L = i;int idx_R = i;
            int count = 0;
            while (true){
                count++;
                if (count > width / 8)break;
                pcl::PointXYZI tmpPt_L = in_cloud_ptr->points[j*width + idx_L];
                pcl::PointXYZI tmpPt_R = in_cloud_ptr->points[j*width + idx_R];
                if (ignore(Point3f(tmpPt_L.x,tmpPt_L.y,tmpPt_L.z))){
                    idx_L--;
                    idx_L = idx_L<0? width + idx_L:idx_L;
                }else{
                    tmpFor = tmpPt_L;
                    break;
                }

                if (ignore(Point3f(tmpPt_R.x,tmpPt_R.y,tmpPt_R.z))){
                    idx_R++;
                    idx_R = idx_R>=width? idx_R - width:idx_R;
                }else{
                    tmpFor = tmpPt_R;
                    break;
                }
            }
            out_cloud_ptr->points[j*width + i] = tmpFor;
        }
    }

}

void terrian_estimate:: computeFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_ori_cloud_ptr,
                                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pre_cloud_ptr,
                                       pcl::PointCloud<MyPointType>::Ptr out_feature_ptr){

    int width = in_ori_cloud_ptr->width;
    int height = in_ori_cloud_ptr->height;
    out_feature_ptr->width = width;
    out_feature_ptr->height = height;
    out_feature_ptr->resize(width * height);

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            int idx = j * width + i;
            pcl::PointXYZI tmpOri = in_ori_cloud_ptr->points[idx];
            pcl::PointXYZI tmpPre = in_pre_cloud_ptr->points[idx];
            out_feature_ptr->points[idx].x = tmpOri.x;
            out_feature_ptr->points[idx].y = tmpOri.y;
            out_feature_ptr->points[idx].z = tmpOri.z;
            out_feature_ptr->points[idx].tagF = false;
            out_feature_ptr->points[idx].wF.f1_Range = comPuteDis3D(Point3f(tmpPre.x, tmpPre.y, tmpPre.z),
                                                                    Point3f(0, 0, 0));//只有在计算range的时候用pre
            out_feature_ptr->points[idx].wF.f2_Intensity = in_ori_cloud_ptr->points[idx].intensity;
        }
    }
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            int idx = j * width + i;
            pcl::PointXYZI tmpOri = in_ori_cloud_ptr->points[idx];
            if (ignore(Point3f(tmpOri.x, tmpOri.y, tmpOri.z)))continue;

            MyPointType tmpL, tmpR, tmpT, tmpB;

            tmpL = i == 0 ? out_feature_ptr->points[j*width + width - 1] : out_feature_ptr->points[j*width + i - 1];
            tmpR = i == (width - 1) ? out_feature_ptr->points[j*width + 0] : out_feature_ptr->points[j*width + i + 1];
            tmpT = j == (height - 1) ? out_feature_ptr->points[j*width + i] : out_feature_ptr->points[(j + 1)*width + i];
            tmpB = j == 0 ? out_feature_ptr->points[j*width + i] : out_feature_ptr->points[(j - 1)*width + i];

            out_feature_ptr->points[idx].wF.f3_Lrange = tmpL.wF.f1_Range - out_feature_ptr->points[idx].wF.f1_Range;
            out_feature_ptr->points[idx].wF.f4_Rrange = tmpR.wF.f1_Range - out_feature_ptr->points[idx].wF.f1_Range;
            out_feature_ptr->points[idx].wF.f5_Trange = tmpT.wF.f1_Range - out_feature_ptr->points[idx].wF.f1_Range;
            out_feature_ptr->points[idx].wF.f6_Brange = tmpB.wF.f1_Range - out_feature_ptr->points[idx].wF.f1_Range;
            out_feature_ptr->points[idx].wF.f7_DisBelow = out_feature_ptr->points[idx].z;

            Point2i tmpIdxPt = computeIndex(Point2f(tmpOri.x,tmpOri.y));

            double tmp = gridLowestH.at<double>(tmpIdxPt.y,tmpIdxPt.x);
            tmp = tmp < tmpOri.z ? tmp:tmpOri.z;
            gridLowestH.at<double>(tmpIdxPt.y,tmpIdxPt.x) = tmp;
        }
    }

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            pcl::PointXYZI tmpI = in_ori_cloud_ptr->points[j*width + i];
            if (ignore(Point3f(tmpI.x,tmpI.y,tmpI.z)))continue;
            Point2i tmpIdxPt = computeIndex(Point2f(tmpI.x,tmpI.y));
            float tmp = tmpI.z - gridLowestH.at<double>(tmpIdxPt.y,tmpIdxPt.x);
            out_feature_ptr->points[j*width + i].wF.f8_HeightInLocal = tmp;
        }
    }
}

void terrian_estimate::genFeatureMat(const pcl::PointCloud<MyPointType>::Ptr in_feature_ptr,
                                     Mat& out_feature_mat){
    int width = in_feature_ptr->width;
    int height = in_feature_ptr->height;
    out_feature_mat = Mat::zeros(width * height,8,CV_32F);
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            int idx = j*width + i;
            MyPointType tmpPt = in_feature_ptr->points[idx];
            out_feature_mat.at<float>(idx,0) = tmpPt.wF.f1_Range;
            out_feature_mat.at<float>(idx,1) = tmpPt.wF.f2_Intensity;
            out_feature_mat.at<float>(idx,2) = tmpPt.wF.f3_Lrange;
            out_feature_mat.at<float>(idx,3) = tmpPt.wF.f4_Rrange;
            out_feature_mat.at<float>(idx,4) = tmpPt.wF.f5_Trange;
            out_feature_mat.at<float>(idx,5) = tmpPt.wF.f6_Brange;
            out_feature_mat.at<float>(idx,6) = tmpPt.wF.f7_DisBelow;
            out_feature_mat.at<float>(idx,7) = tmpPt.wF.f8_HeightInLocal;
        }
    }
}

void terrian_estimate::genTerrian(const Mat in_feature_mat,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                  lidar_msg::terrianPtr out_terrian){

    vector<int> flag;flag.clear();
    for (int i = 0; i < in_feature_mat.rows; ++i) {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        if (ignore(Point3f(tmpI.x,tmpI.y,tmpI.z))){
            flag.push_back(0);
            continue;
        }
        const Mat sample = in_feature_mat.row(i);
        CvDTreeNode* prediction = dtree->predict(sample);
        if (prediction->value >0) {
            flag.push_back(1);
        }else{
            flag.push_back(0);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*in_cloud_ptr, output);
    out_terrian->cloud = output;
    out_terrian->indices = flag;
}

void terrian_estimate::genFloor(const lidar_msg::terrianPtr in_terrian,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr floor_cloud_ptr,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud_ptr){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(in_terrian->cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloudPtr);

    vector<int> tmpIndices = in_terrian->indices;
    for (int i = 0; i < tmpIndices.size(); ++i) {
        if (tmpIndices[i]){
            nofloor_cloud_ptr->push_back(cloudPtr->points[i]);
        }else{
            floor_cloud_ptr->push_back(cloudPtr->points[i]);
        }
    }
    /*
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    inliers->indices = in_terrian->indices;

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud (cloudPtr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*floor_cloud_ptr);
    extract.setNegative(false);
    extract.filter(*nofloor_cloud_ptr);
    */
}

void terrian_estimate::publishCloud(const ros::Publisher* in_publisher,
                                    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr){
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header=_velodyne_header;
    in_publisher->publish(cloud_msg);
}

void terrian_estimate::publishTerrian(const ros::Publisher* in_publisher,
                                      const lidar_msg::terrianPtr in_terrian){
    in_terrian->header = _velodyne_header;
    in_publisher->publish(in_terrian);
    //cout<<in_terrian->header.stamp<<endl;
}

float terrian_estimate::comPuteDis3D(Point3f pt1, Point3f pt2){
    return abs(sqrtf(powf((pt1.x - pt2.x), 2) + powf((pt1.y - pt2.y), 2) + powf((pt1.z - pt2.z), 2)));
}

bool terrian_estimate::ignore(Point3f pt){
    if (abs(pt.x) < 0.0001 && abs(pt.y) < 0.0001 && abs(pt.z) < 0.0001) return true;
    return false;
}

Point2i terrian_estimate::computeIndex(Point2f pt){
    int indexX = pt.x < 0 ? int(pt.x / stepGrid) + BiasX - 1 : int(pt.x / stepGrid) + BiasX;
    int indexY = pt.y < 0 ? int(pt.y / stepGrid) + BiasY - 1 : int(pt.y / stepGrid) + BiasY;
    return Point2i(indexX, indexY);
}


