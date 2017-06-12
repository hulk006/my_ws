#include <ros/ros.h>
#include "lidar_msg/clusterArray.h"
#include "track.h"
#include "multiClassifier.h"
#include "drawing.h"
using std::string;

ros::Publisher vel_pub;
ros::Publisher pc_pub;
Robosense::MultiClassifier mcf;
Robosense::Tracking MM;

void classifyTrackCallback(const lidar_msg::clusterArray& msg)
{
    pcl::PointCloud<pcl::PointXYZI> pCloud;
    std::vector<Robosense::SegMsg> segmsg2_0;
    MM.convertMsgForTracking(msg,pCloud,segmsg2_0);
    MM.trackSegMain(segmsg2_0);
    for(int i = 0; i<MM.target_current_.size();i++)
    {
        std::vector<double> tmprob;
        std::vector<double> label;
        std::vector<pcl::PointCloud<pcl::PointXYZI> >vec;
        vec.push_back(MM.target_current_[i].pointcloud_);
        mcf.Classify(vec,tmprob,label);
        MM.target_current_[i].classify_ = label[0];
    }
    Robosense::Drawing drawing;
    drawing.drawInRviz(vel_pub,pc_pub,MM, pCloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seg_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  std::string modelfile,rangefile, obdfile, mapfile;
  std::vector<string> files;
  std::string pkgPath = ros::package::getPath("classify_sdk");
  private_nh.param("modelfile",modelfile,std::string(pkgPath+"/svm_model/traincarpedbkg.txt.model"));
  private_nh.param("rangefile",rangefile,std::string(pkgPath+"/svm_model/traincarpedbkg.txt.range"));
  private_nh.param("obdfile", obdfile, std::string(pkgPath+"/../data/obd.txt"));
  private_nh.param("mapfile", mapfile, std::string(pkgPath+"/../data/0503vel.txt"));
  mcf.setModelRange(modelfile,rangefile);
  MM.setVelFilePath(obdfile, mapfile, 0);

  ros::Subscriber seg_sub = node.subscribe("clusters", 10, classifyTrackCallback);
  vel_pub = node.advertise<visualization_msgs::MarkerArray>("velocity",10);
  pc_pub = node.advertise<sensor_msgs::PointCloud2>("pc_cloud",10);

  ros::spin();


  return 0;
}
