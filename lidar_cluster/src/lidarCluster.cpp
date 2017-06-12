//
// Created by mjj on 17-5-2.
//

#include "../include/lidarCluster.h"

lidar_cluster::lidar_cluster(ros::NodeHandle node,
                             ros::NodeHandle private_nh){
    string topic = private_nh.param("topic",string("terrian"));

    sub_tracklets = node.subscribe(topic,10,&lidar_cluster::terrianCallback,(lidar_cluster*)this);

    pub_colors = node.advertise<sensor_msgs::PointCloud2>("clusterColor",10);
    pub_grid = node.advertise<sensor_msgs::Image>("grid",1);
    pub_clusters = node.advertise<lidar_msg::clusterArray>("clusters",10);

    string pwd = ros::package::getPath("lidar_cluster");
    string model_str = private_nh.param("model",pwd + "/model/dtree.xml");

    in_clip_min_height = private_nh.param("in_clip_min_height",float(-1.3));
    in_clip_max_height = private_nh.param("in_clip_max_height",float(0.5));

    gWidth = private_nh.param("gWidth",float(50.));
    gHeight = private_nh.param("gHeight",float(50.));
    miniGrid = private_nh.param("miniGrid",float(0.15));
    dilation_size = private_nh.param("dilation_size",int(1));

    gridMiH = -500;gridMaH = 500;

    //计算栅格长宽
    gridH = gHeight / miniGrid; gridW = gWidth / miniGrid;
    //用于分割的栅格图
    grid = Mat::zeros(gridH, gridW, CV_8UC1);

    colorsNum = 200;
    generateColors(_colors,colorsNum);
}

void lidar_cluster::genClusters(const Mat label,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                lidar_msg::clusterArrayPtr clusters){
    double min,max;
    minMaxIdx(label,&min,&max);
    vector<pcl::PointCloud<pcl::PointXYZI> > tmpPCDs;
    tmpPCDs.resize(max);

    for (int i = 0; i < in_cloud_ptr->size(); ++i) {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        Point2i tmpIndex = trans(Point2f(tmpI.x,tmpI.y));
        if (tmpIndex.x >= 0 && tmpIndex.x < gridW && tmpIndex.y >= 0 && tmpIndex.y < gridH)
        {
            int index = label.at<int>(tmpIndex.y,tmpIndex.x);
            if (index <= 0)continue;
            tmpPCDs[index - 1].push_back(tmpI);
        }
    }

    sensor_msgs::PointCloud2 output;
    for (int i = 0; i < tmpPCDs.size(); ++i) {
        lidar_msg::clusterPtr tmpCluster(new lidar_msg::cluster);
        Point2f centerPt;
        centerPt.x = 0. ; centerPt.y = 0. ;
        for (int j = 0; j < tmpPCDs[i].size(); ++j) {
            pcl::PointXYZI tmpI = tmpPCDs[i].points[j];
            centerPt.x += tmpI.x;
            centerPt.y += tmpI.y;
            tmpCluster->x.push_back(tmpI.x);
            tmpCluster->y.push_back(tmpI.y);
            tmpCluster->z.push_back(tmpI.z);
            tmpCluster->i.push_back(tmpI.intensity);
        }
        centerPt.x /= tmpPCDs[i].size();
        centerPt.y /= tmpPCDs[i].size();
        tmpCluster->centerPtx = centerPt.x;
        tmpCluster->centerPty = centerPt.y;
        tmpCluster->inROI = true;
        clusters->clusters.push_back(*tmpCluster);
        /*
        pcl::toROSMsg(tmpPCDs[i],output);
        lidar_msg::clusterPtr tmpCluster(new lidar_msg::cluster);
        tmpCluster->cloud = output;
        clusters->clusters.push_back(*tmpCluster);
        */
    }
}

int lidar_cluster::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
             (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
            c = !c;
    }
    return c;
}

void lidar_cluster::colors(const lidar_msg::clusterArrayPtr clusters,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_color_cloud_ptr){
    out_color_cloud_ptr->clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpPCD(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = 0; i < clusters->clusters.size(); ++i) {
        if (clusters->clusters[i].x.size()<10)continue;
        for (int j = 0; j < clusters->clusters[i].x.size(); ++j) {
            pcl::PointXYZRGB tmpRGB;
            tmpRGB.x = clusters->clusters[i].x[j];
            tmpRGB.y = clusters->clusters[i].y[j];
            tmpRGB.z = clusters->clusters[i].z[j];
            if (clusters->clusters[i].inROI && i!=clusters->clusters.size() - 1){
                tmpRGB.r = _colors[i][0];
                tmpRGB.g = _colors[i][1];
                tmpRGB.b = _colors[i][2];
            }else{
                tmpRGB.r = 0;
                tmpRGB.g = 0;
                tmpRGB.b = 0;
            }

            out_color_cloud_ptr->push_back(tmpRGB);
        }
        /*
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(clusters->clusters[i].cloud,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*tmpPCD);
        if(tmpPCD->size() <10)continue;
        for (int j = 0; j < tmpPCD->size(); ++j) {
            pcl::PointXYZRGB tmpRGB;
            tmpRGB.x = tmpPCD->points[j].x;
            tmpRGB.y = tmpPCD->points[j].y;
            tmpRGB.z = tmpPCD->points[j].z;
            tmpRGB.r = _colors[i][0];
            tmpRGB.g = _colors[i][1];
            tmpRGB.b = _colors[i][2];
            out_color_cloud_ptr->push_back(tmpRGB);
        }
        */
    }

    for (int i = 0; i < ROI.size(); ++i) {
        pcl::PointXYZRGB tmpRGB;
        tmpRGB.x = ROI[i].x;
        tmpRGB.y = ROI[i].y;
        tmpRGB.z = 0;
        tmpRGB.r = 255;
        tmpRGB.g = 255;
        tmpRGB.b = 255;
        out_color_cloud_ptr->push_back(tmpRGB);
    }
}

void lidar_cluster::genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr){
    for (int i = 0; i < in_cloud_ptr->size(); ++i) {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        Point2i tmpIdx = trans(Point2f(tmpI.x,tmpI.y));
        if(tmpIdx.x >= 0 && tmpIdx.x < gridW && tmpIdx.y >= 0 && tmpIdx.y < gridH){
            grid.at<uchar>(tmpIdx.y,tmpIdx.x) = 1;
        }
    }
}

Point2i lidar_cluster::trans(Point2f pt){
    return Point2i(int(pt.x / miniGrid) + gridW / 2, int(pt.y / miniGrid) + gridH / 2);
}

void lidar_cluster::publishclusterArray(const ros::Publisher* in_publisher, const lidar_msg::clusterArrayPtr in_clusterArray)
{
    in_clusterArray->header = _velodyne_header;
    in_publisher->publish(in_clusterArray);
    //cout<<in_clusterArray->header.stamp<<endl;
}

void lidar_cluster::publishImage(const ros::Publisher* in_publisher, const Mat inMat)
{
    sensor_msgs::Image mat_msg;
    cv_bridge::CvImage out_msg;
    out_msg.header   = _velodyne_header; // Same timestamp and tf frame as input image
    if(inMat.type() == CV_8UC1)
    {
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
    }else{
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
    }

    out_msg.image    = inMat;

    in_publisher->publish(out_msg);
}

void lidar_cluster::publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header=_velodyne_header;
    in_publisher->publish(cloud_msg);
}

void lidar_cluster::publishColorCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header=_velodyne_header;
    in_publisher->publish(cloud_msg);
}

void lidar_cluster::terrianCallback(const lidar_msg::terrian& msg){
    _velodyne_header =msg.header;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg.cloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    ROI.clear();
    if (msg.ROIptx.size()>0){
        for (int i = 0; i < msg.ROIptx.size(); ++i) {
            ROI.push_back(Point2f(msg.ROIptx[i],msg.ROIpty[i]));
        }
    }
    ignorPCD.clear();
    //-----------------这一段可以优化

    vector<int> indices = msg.indices;

    for (int i = 0; i < indices.size(); ++i) {
        pcl::PointXYZI tmpI = cloud->points[i];
        if (indices[i])nofloor_cloud_ptr->push_back(tmpI);
        else{
 //           ignorPCD.push_back(tmpI);
        }
    }

    pushFrame(nofloor_cloud_ptr);
}

void lidar_cluster::pushFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr)
{
    grid.setTo(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    clipCloud(in_cloud_ptr,clipped_cloud_ptr,in_clip_min_height,in_clip_max_height);
    genGrid(clipped_cloud_ptr);

    //对栅格进行膨胀
    Mat element = getStructuringElement( MORPH_RECT,
                                         Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         Point( dilation_size, dilation_size ) );
    dilate(grid,grid,element);
    //连通域
    Mat label;
    icvprCcaBySeedFill(grid,label);

    lidar_msg::clusterArrayPtr clusters(new lidar_msg::clusterArray);

    genClusters(label,clipped_cloud_ptr,clusters);
    if (ROI.size() > 0){
        for (int j = 0; j < clusters->clusters.size(); ++j) {
            float* vertx = new float[ROI.size()];
            float* verty = new float[ROI.size()];
            for (int i = 0; i < ROI.size(); ++i) {
                vertx[i] = ROI[i].x;
                verty[i] = ROI[i].y;
            }

            if (!pnpoly(ROI.size(),vertx,verty,clusters->clusters[j].centerPtx,clusters->clusters[j].centerPtx))
            {
                clusters->clusters[j].inROI = false;
            }
        }
    }

    lidar_msg::clusterPtr ignorPtr(new lidar_msg::cluster);
    for (int i = 0; i < ignorPCD.size(); ++i) {
        pcl::PointXYZI tmpI = ignorPCD.points[i];
        ignorPtr->x.push_back(tmpI.x);
        ignorPtr->y.push_back(tmpI.y);
        ignorPtr->z.push_back(tmpI.z);
        ignorPtr->i.push_back(tmpI.intensity);
    }
    ignorPtr->centerPtx = 0.;
    ignorPtr->centerPty = 0.;
    ignorPtr->inROI = false;
    clusters->clusters.push_back(*ignorPtr);

    colors(clusters,colored_clustered_cloud_ptr);

    publishColorCloud(&pub_colors,colored_clustered_cloud_ptr);
    publishclusterArray(&pub_clusters,clusters);
    grid *= 255;
    publishImage(&pub_grid,grid);
}


void lidar_cluster::icvprCcaBySeedFill(const Mat& _binImg, Mat& _lableImg){
    // connected component analysis (4-component)
    // use seed filling algorithm
    // 1. begin with a foreground pixel and push its foreground neighbors into a stack;
    // 2. pop the top pixel on the stack and label it with the same label until the stack is empty
    //
    // foreground pixel: _binImg(x,y) = 1
    // background pixel: _binImg(x,y) = 0

    if (_binImg.empty() ||
        _binImg.type() != CV_8UC1)
    {
        return ;
    }

    Mat edge_binImg = Mat::zeros(_binImg.rows + 2,_binImg.cols + 2,CV_8UC1);

    _binImg.copyTo(edge_binImg(Rect(1,1,_binImg.cols,_binImg.rows)));

    _lableImg.release() ;
    edge_binImg.convertTo(_lableImg, CV_32SC1) ;

    int label = 1 ;  // start by 2

    int rows = _binImg.rows - 1 ;
    int cols = _binImg.cols - 1 ;

    for (int i = 1; i < rows-1; i++)
    {
        int* data= _lableImg.ptr<int>(i) ;
        for (int j = 1; j < cols-1; j++)
        {
            if (data[j] == 1)
            {
                std::stack<std::pair<int,int> > neighborPixels ;
                neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>
                ++label ;  // begin with a new label
                while (!neighborPixels.empty())
                {
                    // get the top pixel on the stack and label it with the same label
                    std::pair<int,int> curPixel = neighborPixels.top() ;
                    int curX = curPixel.first ;
                    int curY = curPixel.second ;
                    _lableImg.at<int>(curX, curY) = label ;

                    // pop the top pixel
                    neighborPixels.pop() ;

                    // push the 4-neighbors (foreground pixels)
                    if (_lableImg.at<int>(curX, curY-1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX, curY+1) == 1)
                    {// right pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
                    }
                    if (_lableImg.at<int>(curX-1, curY) == 1)
                    {// up pixel
                        neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
                    }
                    if (_lableImg.at<int>(curX+1, curY) == 1)
                    {// down pixel
                        neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
                    }
                    /*
                    if (_lableImg.at<int>(curX-1, curY-1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX+1, curY-1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX-1, curY+1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX+1, curY+1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    */
                }
            }
        }
    }

    Mat tmp = _lableImg.clone();
    _lableImg.release();
    _lableImg = Mat::zeros(tmp.rows - 2,tmp.cols - 2,CV_32SC1);
    tmp(Rect(1,1,_lableImg.cols,_lableImg.rows)).copyTo(_lableImg);
    tmp.release();
}

void lidar_cluster::clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                              float in_min_height, float in_max_height){
    out_cloud_ptr->points.clear();
    for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
    {
        if (in_cloud_ptr->points[i].z >= in_min_height &&
            in_cloud_ptr->points[i].z <= in_max_height)
        {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }else{
            ignorPCD.push_back(in_cloud_ptr->points[i]);
        }
    }
}

