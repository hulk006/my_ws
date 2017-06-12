//
// Created by mjj on 17-5-26.
//
#include "roboCluster.h"

namespace Robosense{
    roboCluster::roboCluster(){
        initFlag = false;
    }

    void roboCluster::roboClusterInit(std::string str,double maxHeight,double maxWidth,double miniGrid,
                                      int dilaSize,double clip_min_height,double clip_max_height){
        stepGrid = 2.;
        maxH = maxHeight;
        maxW = maxWidth;

        BiasY = int(maxH / stepGrid) + 1;
        BiasX = int(maxW / stepGrid) + 1;
        gridLowestH = cv::Mat::zeros(BiasY * 2, BiasX * 2, CV_64F);
        dtree = new CvDTree;
        dtree->load(str.c_str());

        gWidth = maxWidth;
        gHeight = maxHeight;
        miniG = miniGrid;
        in_clip_min_height = clip_min_height;
        in_clip_max_height = clip_max_height;
        dilation_size = dilaSize;

        //计算栅格长宽
        gridH = gHeight / miniGrid; gridW = gWidth / miniGrid;
        //用于分割的栅格图
        grid = cv::Mat::zeros(gridH, gridW, CV_8UC1);
        initFlag = true;
    }

    void roboCluster::cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud_ptr,
                              std::vector<pcl::PointCloud<pcl::PointXYZI> >& out_clusters){
        if (!initFlag){
            std::cout<<"must init before cluster!"<<std::endl;
            return;
        }
        ignorePts.clear();
        //--------------首先进行地面点去除----------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr sorted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr preProd_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<MyPointType>::Ptr feature_cloud_ptr(new pcl::PointCloud<MyPointType>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        cv::Mat featureMat;

        //--------检测是否是16线的有序点云，如果不是需要转成有序进行操作
        if (in_cloud_ptr->height == BEAMSNUM_16){
            *sorted_cloud_ptr = *in_cloud_ptr;
        }else{
            tranSort(in_cloud_ptr,sorted_cloud_ptr);
        }

        gridLowestH.setTo(INT_MAX);

        preProcFrame(sorted_cloud_ptr,preProd_cloud_ptr);
        computeFeature(sorted_cloud_ptr,preProd_cloud_ptr,feature_cloud_ptr);
        genFeatureMat(feature_cloud_ptr,featureMat);
        genTerrian(featureMat,sorted_cloud_ptr,ground_cloud_ptr,noground_cloud_ptr);

        //-----------------------去除地面点后，进行分割
        pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        cv::Mat label;
        grid.setTo(0);

        clipCloud(noground_cloud_ptr,clipped_cloud_ptr,in_clip_min_height,in_clip_max_height);
        genGrid(clipped_cloud_ptr,gridH,gridW,grid);

        //对栅格进行膨胀
        cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                                     cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                     cv::Point( dilation_size, dilation_size ) );
        dilate(grid,grid,element);
        //连通域
        icvprCcaBySeedFill(grid,label);
        genClusters(label,clipped_cloud_ptr,out_clusters);

        out_clusters.push_back(ignorePts);
    }

    void roboCluster::tranSort(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr){
        int height = BEAMSNUM_16;
        int width = in_cloud_ptr->size() / height;
        out_cloud_ptr->height = height;
        out_cloud_ptr->width = width;
        out_cloud_ptr->resize(height * width);

        int index = 0;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                out_cloud_ptr->points[j * width + i] = in_cloud_ptr->points[index];
                index++;
            }
        }
    }

    void roboCluster::preProcFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
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
                if (!ignore(cv::Point3f(tmpPt.x,tmpPt.y,tmpPt.z))){
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
                    if (ignore(cv::Point3f(tmpPt_L.x,tmpPt_L.y,tmpPt_L.z))){
                        idx_L--;
                        idx_L = idx_L<0? width + idx_L:idx_L;
                    }else{
                        tmpFor = tmpPt_L;
                        break;
                    }

                    if (ignore(cv::Point3f(tmpPt_R.x,tmpPt_R.y,tmpPt_R.z))){
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

    void roboCluster::computeFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_ori_cloud_ptr,
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
                out_feature_ptr->points[idx].wF.f1_Range = comPuteDis3D(cv::Point3f(tmpPre.x, tmpPre.y, tmpPre.z),
                                                                        cv::Point3f(0, 0, 0));//只有在计算range的时候用pre
                out_feature_ptr->points[idx].wF.f2_Intensity = in_ori_cloud_ptr->points[idx].intensity;
            }
        }
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                int idx = j * width + i;
                pcl::PointXYZI tmpOri = in_ori_cloud_ptr->points[idx];
                if (ignore(cv::Point3f(tmpOri.x, tmpOri.y, tmpOri.z)))continue;

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

                cv::Point2i tmpIdxPt = computeIndex(cv::Point2f(tmpOri.x,tmpOri.y));

                double tmp = gridLowestH.at<double>(tmpIdxPt.y,tmpIdxPt.x);
                tmp = tmp < tmpOri.z ? tmp:tmpOri.z;
                gridLowestH.at<double>(tmpIdxPt.y,tmpIdxPt.x) = tmp;
            }
        }

        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                pcl::PointXYZI tmpI = in_ori_cloud_ptr->points[j*width + i];
                if (ignore(cv::Point3f(tmpI.x,tmpI.y,tmpI.z)))continue;
                cv::Point2i tmpIdxPt = computeIndex(cv::Point2f(tmpI.x,tmpI.y));
                float tmp = tmpI.z - gridLowestH.at<double>(tmpIdxPt.y,tmpIdxPt.x);
                out_feature_ptr->points[j*width + i].wF.f8_HeightInLocal = tmp;
            }
        }
    }

    void roboCluster::genFeatureMat(const pcl::PointCloud<MyPointType>::Ptr in_feature_ptr,cv::Mat& out_feature_mat){
        int width = in_feature_ptr->width;
        int height = in_feature_ptr->height;
        out_feature_mat = cv::Mat::zeros(width * height,8,CV_32F);
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                int idx = j*width + i;
                MyPointType tmpPt = in_feature_ptr->points[idx];
                //----------------
                out_feature_mat.at<float>(idx,0) = tmpPt.wF.f1_Range;
                out_feature_mat.at<float>(idx,1) = tmpPt.wF.f2_Intensity;
                out_feature_mat.at<float>(idx,2) = tmpPt.wF.f3_Lrange;
                out_feature_mat.at<float>(idx,3) = tmpPt.wF.f4_Rrange;
                out_feature_mat.at<float>(idx,4) = tmpPt.wF.f5_Trange;
                out_feature_mat.at<float>(idx,5) = tmpPt.wF.f6_Brange;
                out_feature_mat.at<float>(idx,6) = tmpPt.wF.f7_DisBelow;
                out_feature_mat.at<float>(idx,7) = tmpPt.wF.f8_HeightInLocal;
                //-----------------
            }
        }
    }

    void roboCluster::genTerrian(const cv::Mat in_feature_mat,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud_ptr){
        ground_cloud_ptr->clear();
        noground_cloud_ptr->clear();
        for (int i = 0; i < in_feature_mat.rows; ++i) {
            pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
            if (ignore(cv::Point3f(tmpI.x,tmpI.y,tmpI.z))){
                continue;
            }
            const cv::Mat sample = in_feature_mat.row(i);
            CvDTreeNode* prediction = dtree->predict(sample);
            if (prediction->value >0) {
                noground_cloud_ptr->push_back(tmpI);
            }else{
                ground_cloud_ptr->push_back(tmpI);
            }
        }
    }

    float roboCluster::comPuteDis3D(cv::Point3f pt1, cv::Point3f pt2){
        return fabs(sqrtf(powf((pt1.x - pt2.x), 2) + powf((pt1.y - pt2.y), 2) + powf((pt1.z - pt2.z), 2)));
    }

    bool roboCluster::ignore(cv::Point3f pt){
        if (abs(pt.x) < 0.0001 && abs(pt.y) < 0.0001 && abs(pt.z) < 0.0001) return true;
        return false;
    }

    cv::Point2i roboCluster::computeIndex(cv::Point2f pt){
        int indexX = pt.x < 0 ? int(pt.x / stepGrid) + BiasX - 1 : int(pt.x / stepGrid) + BiasX;
        int indexY = pt.y < 0 ? int(pt.y / stepGrid) + BiasY - 1 : int(pt.y / stepGrid) + BiasY;
        return cv::Point2i(indexX, indexY);
    }

    void roboCluster::genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                          const int gridH,const int gridW,cv::Mat& grid){
        for (int i = 0; i < in_cloud_ptr->size(); ++i) {
            pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
            cv::Point2i tmpIdx = trans(cv::Point2f(tmpI.x,tmpI.y));
            if(tmpIdx.x >= 0 && tmpIdx.x < gridW && tmpIdx.y >= 0 && tmpIdx.y < gridH){
                grid.at<uchar>(tmpIdx.y,tmpIdx.x) = 1;
            }
        }
    }

    cv::Point2i roboCluster::trans(cv::Point2f pt){
        return cv::Point2i(int(pt.x / miniG) + gridW / 2, int(pt.y / miniG) + gridH / 2);
    }

    void roboCluster::clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
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
                ignorePts.push_back(in_cloud_ptr->points[i]);
            }
        }
    }

    void roboCluster::icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg){

        if (_binImg.empty() ||
            _binImg.type() != CV_8UC1)
        {
            return ;
        }

        cv::Mat edge_binImg = cv::Mat::zeros(_binImg.rows + 2,_binImg.cols + 2,CV_8UC1);
        _binImg.copyTo(edge_binImg(cv::Rect(1,1,_binImg.cols,_binImg.rows)));
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
                    }
                }
            }
        }

        cv::Mat tmp = _lableImg.clone();
        _lableImg.release();
        _lableImg = cv::Mat::zeros(tmp.rows - 2,tmp.cols - 2,CV_32SC1);
        tmp(cv::Rect(1,1,_lableImg.cols,_lableImg.rows)).copyTo(_lableImg);
        tmp.release();
    }

    void roboCluster::genClusters(const cv::Mat label, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                  std::vector<pcl::PointCloud<pcl::PointXYZI> >& out_clusters){
        double min,max;
        minMaxIdx(label,&min,&max);
        out_clusters.resize(max);

        for (int i = 0; i < in_cloud_ptr->size(); ++i) {
            pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
            cv::Point2i tmpIndex = trans(cv::Point2f(tmpI.x,tmpI.y));
            if (tmpIndex.x >= 0 && tmpIndex.x < gridW && tmpIndex.y >= 0 && tmpIndex.y < gridH)
            {
                int index = label.at<int>(tmpIndex.y,tmpIndex.x);
                if (index <= 0)continue;
                out_clusters[index - 1].push_back(tmpI);
            }
        }
    }

}