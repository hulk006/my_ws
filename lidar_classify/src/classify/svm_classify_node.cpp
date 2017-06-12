#include <ros/ros.h>
#include "classifyTrack.h"
using std::string;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seg_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  std::string modelfile, rangefile, obdfile, mapfile;
  std::vector<string> files;
  std::string pkgPath = ros::package::getPath("lidar_classify");
  private_nh.param("modelfile",modelfile,std::string(pkgPath+"/svm_model/traincarpedbkg.txt.model"));
  private_nh.param("rangefile",rangefile,std::string(pkgPath+"/svm_model/traincarpedbkg.txt.range"));
  private_nh.param("obdfile", obdfile, std::string(pkgPath+"/../data/obd.txt"));
  private_nh.param("mapfile", mapfile, std::string(pkgPath+"/../data/0503vel.txt"));
  files.push_back(modelfile);
  files.push_back(rangefile);
  files.push_back(obdfile);
  files.push_back(mapfile);
  ClassifyTrack ct(node,private_nh,files);
  ros::spin();


  return 0;
}
