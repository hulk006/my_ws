//
// Created by mjj on 17-5-26.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "roboCluster.h"
#include "lidar_msg/clusterArray.h"
#include "lidar_msg/cluster.h"

ros::Publisher pub_clusterColor;
ros::Publisher pub_clusters;

Robosense::roboCluster CL;

std_msgs::Header _header;
std::vector<cv::Scalar> _colors;

void fullscanCallback(const sensor_msgs::PointCloud2 msg){
    _header = msg.header;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > out_clusters;
    lidar_msg::clusterArrayPtr clusters(new lidar_msg::clusterArray);

    CL.cluster(cloud,out_clusters);

    for (int i = 0; i < out_clusters.size(); ++i) {
        //if (out_clusters.size() < 30) continue;
        lidar_msg::clusterPtr cluster(new lidar_msg::cluster);
        cv::Point3f centerPt(0.,0.,0.);

        for (int j = 0; j < out_clusters[i].size(); ++j) {
            if (out_clusters[i].size() <= 0)continue;
            pcl::PointXYZI tmpI = out_clusters[i].points[j];
            pcl::PointXYZRGB tmpRGB;
            tmpRGB.x = tmpI.x;tmpRGB.y = tmpI.y;tmpRGB.z = tmpI.z;
            tmpRGB.r = _colors[i][0];
            tmpRGB.g = _colors[i][1];
            tmpRGB.b = _colors[i][2];
            color_cloud_ptr->push_back(tmpRGB);

            cluster->x.push_back(tmpI.x);
            cluster->y.push_back(tmpI.y);
            cluster->z.push_back(tmpI.z);
            cluster->i.push_back(tmpI.intensity);
            centerPt.x += tmpI.x;
            centerPt.y += tmpI.y;
            centerPt.z += tmpI.z;
        }
        centerPt.x /= out_clusters[i].size();
        centerPt.y /= out_clusters[i].size();
        cluster->centerPtx = centerPt.x;
        cluster->centerPty = centerPt.y;
        cluster->inROI = false;
        clusters->clusters.push_back(*cluster);
    }
    pub_clusters.publish(clusters);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*color_cloud_ptr, output);
    output.header = _header;
    pub_clusterColor.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ros::Subscriber sub_fullscan = node.subscribe("fullscan",10,fullscanCallback);
    pub_clusterColor = node.advertise<sensor_msgs::PointCloud2>("clusterColor",10);
    pub_clusters = node.advertise<lidar_msg::clusterArray>("clusters",10);

    std::string pwd = "/home/mjj/catkin_ws/src/lidar_cluster";
    std::string model_str = private_nh.param("model",pwd + "/model/dtree.xml");

    CL.roboClusterInit(model_str);
    cv::generateColors(_colors,200);

    ros::spin();

    return 0;
}
