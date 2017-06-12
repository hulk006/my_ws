#include "xgbclassifyTrack.h"
#include "drawing.h"

using namespace Robosense;
xgbClassifyTrack::xgbClassifyTrack(ros::NodeHandle node, ros::NodeHandle private_nh)

{
    seg_sub = node.subscribe("clusters", 10, &xgbClassifyTrack::classifyTrackCallback,(xgbClassifyTrack*)this);

    vel_pub = node.advertise<visualization_msgs::MarkerArray>("velocity_xgb",10);
    pc_pub = node.advertise<sensor_msgs::PointCloud2>("pc_cloud_xgb",10);
}

void xgbClassifyTrack::initClass()
{
    if(MM.target_current_.size() == 0)
        return;
    for(int i = 0; i<MM.target_current_.size();i++)
    {
        Robosense::SegResult cube = MM.target_current_[i].cube_;
        float len = (cube.max_x - cube.min_x) > (cube.max_y - cube.min_y) ? (cube.max_x - cube.min_x): (cube.max_y - cube.min_y);
        float width = (cube.max_x - cube.min_x) < (cube.max_y - cube.min_y) ? (cube.max_x - cube.min_x): (cube.max_y - cube.min_y);
        float height = cube.max_z - cube.min_z;
        float centerX = (cube.max_x + cube.min_x)/2;
        if((len > 2)&&(width < 3.5)&&(centerX>-2)&&(centerX<20))
            MM.target_current_[i].classify_ = 1;
        if((len < 1.5)&&(height < 2))
            MM.target_current_[i].classify_ = 3;
        else
            MM.target_current_[i].classify_ = 2;
    }

}

void xgbClassifyTrack::classifyTrackCallback(const lidar_msg::clusterArray& msg)
{
    clock_t start = clock();

    pcl::PointCloud<pcl::PointXYZI> pCloud;
   // msgConvertion(msg,seg_vec,pCloud);

    std::vector<Robosense::SegMsg> segmsg2_0;
    //将接受到的信息存入segmsg2_0
    MM.convertMsgForTracking(msg,pCloud,segmsg2_0);

    //获得可以追踪到的目标
    MM.trackSegMain(segmsg2_0);

    initClass();

    for(int i = 0; i<MM.target_current_.size();i++)
    {
		pcl::PointCloud<pcl::PointXYZI> cloudtmp;
        cloudtmp=MM.target_current_[i].pointcloud_;

        //feature
        //测试feature
        classifyWithXgboost myxgboost;
        Feature_ testFeature;
        myxgboost.featureExtraction(cloudtmp,testFeature);

        vector<float> testFeatureVectmp;
        myxgboost.feature2Vec(testFeature,testFeatureVectmp);

        vector<float > testResult;
        myxgboost.xgboostClassification(testFeature,testResult);

        //概率转为label
        int label=0;
        vector<float >::iterator biggest = std::max_element(testResult.begin(), testResult.end());
        int elementNum=std::distance(testResult.begin(), biggest);
        if (elementNum==0){
            label=4;//backg
        } else if(elementNum==2){
            label=3;//ped
        } else if (elementNum==1){
            label=1;//car
        } else if(elementNum==3){
            label = 2;//truck
        }
        if(MM.target_current_[i].classify_ == 1 && label == 4)
            MM.target_current_[i].classify_ = 1;
        else
            MM.target_current_[i].classify_ = label;
        //MM.target_real[i].label_probability=testResult[elementNum];

        std::cout<<"xgb_label:"<<label<<std::endl;

    }
    Robosense::Drawing drawing;
    drawing.drawInRviz(vel_pub,pc_pub,MM, pCloud);


     clock_t end1 = clock();
    std::cout << "the cost1 time is: " << (double)(end1 - start)/1000 << " ms" << std::endl;
}

xgbClassifyTrack::~xgbClassifyTrack() {

}


