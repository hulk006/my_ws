//
// Created by mjj on 17-5-26.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include "Tracking.h"
#include "drawRviz.h"
#include "roboCluster.h"

template<typename T>
std::string num2str(T num)
{
    std::stringstream ss;
    std::string st;
    ss << num;
    ss >> st;
    return st;
}

ros::Publisher pub_track_cloud;//显示跟踪点云
ros::Publisher pub_track_info;//显示框，文字，箭头等

Robosense::roboCluster clu;
Tracking tra;
drawRviz dra;

std_msgs::Header _header;
std::vector<cv::Scalar> _colors;

void publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr);
void publishColorCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);
void transSensorMsgToPCloud(const sensor_msgs::PointCloud2& msg,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr);

void fullscanCallback(const sensor_msgs::PointCloud2 msg){
    _header = msg.header;
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    transSensorMsgToPCloud(msg,in_cloud_ptr);

    clock_t start = clock();
    std::vector<pcl::PointCloud<pcl::PointXYZI> > clusters;
    clu.cluster(in_cloud_ptr,clusters);

    std::cout<<"cluster cost time: "<<clock() - start<<std::endl;
    start = clock();

    double timeStam = msg.header.stamp.toSec();
    tra.pushNewFrame(clusters,timeStam);
    std::cout<<"track cost time: "<<clock() - start<<std::endl;
    start = clock();

    std::vector<precision_tracking::Tracker> trackers = tra.trackers;
    dra.drawTrack(pub_track_info,pub_track_cloud,_header,trackers);
    std::cout<<"draw rviz cost time: "<<clock() - start<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub_fullscan = node.subscribe("fullscan",10,fullscanCallback);
    pub_track_cloud = node.advertise<sensor_msgs::PointCloud2>("tracker_cloud",10);
    pub_track_info = node.advertise<visualization_msgs::MarkerArray>("tracker_info",10);

    std::string modelPath = "/home/mjj/catkin_ws/src/lidar_cluster/model/dtree.xml";
    clu.roboClusterInit(modelPath);

    std::string dataPath = "/home/mjj/data/lidar/0503/";
    std::string strICPPath = dataPath + "0503vel.txt";
    std::string strOBDPath = dataPath + "obd.txt";
    tra.readICP(strICPPath);
    tra.readOBD(strOBDPath);
    generateColors(_colors,100);

    ros::spin();

    return 0;
}

void transSensorMsgToPCloud(const sensor_msgs::PointCloud2& msg,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*out_cloud_ptr);
}

void readOBD(std::string obdPath, std::vector<std::vector<double> >& self_velocity){
    std::ifstream ifile;
    ifile.open(obdPath.c_str());
    assert(ifile.is_open());

    self_velocity.clear();
    std::string str;
    while (!ifile.eof())
    {
        getline(ifile, str);
        std::istringstream infile(str);
        std::vector<double> tmp_self_velocity;
        std::string tmp_c;
        infile >> tmp_c;
        double tmp = atof(tmp_c.c_str()) / 1.e6;
        tmp_self_velocity.push_back(tmp);
        infile >> tmp_c;
        tmp_self_velocity.push_back(atof(tmp_c.c_str()));
        self_velocity.push_back(tmp_self_velocity);
    }
}
void readICP(std::string ocpPath, std::vector<std::vector<double> >& self_velocity){
    std::ifstream ifile;
    ifile.open(ocpPath.c_str());
    assert(ifile.is_open());

    self_velocity.clear();
    std::string str;
    while (!ifile.eof())
    {
        getline(ifile, str);
        std::istringstream infile(str);
        std::vector<double> tmp_self_velocity;
        std::string tmp_c;
        infile >> tmp_c;
        double tmp = atof(tmp_c.c_str());
        double aa = (int)tmp;
        double bb = tmp - (int)tmp;
        tmp = aa + bb * 1000;
        tmp_self_velocity.push_back(tmp);
        for (int i = 1; i < 6; ++i) {
            infile >> tmp_c;
        }

        infile>>tmp_c;
        double yaw = atof(tmp_c.c_str());
        infile>>tmp_c;
        double tmp_vx = atof(tmp_c.c_str());
        infile>>tmp_c;
        double tmp_vy = atof(tmp_c.c_str());

        double actual_vy = tmp_vx * cos(yaw) + tmp_vy * sin(yaw);
        double actual_vx = -1 * tmp_vx * sin(yaw) + tmp_vy * cos(yaw);

        tmp_self_velocity.push_back(actual_vx);
        tmp_self_velocity.push_back(actual_vy);
        self_velocity.push_back(tmp_self_velocity);
    }
}

void publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr){
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header=_header;
    in_publisher->publish(cloud_msg);
}
void publishColorCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr){
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header=_header;
    in_publisher->publish(cloud_msg);
}