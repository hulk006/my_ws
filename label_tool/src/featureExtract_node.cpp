#include"track_label.h"
#include "featureExtract.h"
#include<iomanip>
#include "std_msgs/Int16.h"
#include"std_msgs/Int16MultiArray.h"
#include"std_msgs/String.h"
#include "ShapeEstimator.h"

std::vector<string> readTxt(string file)
{
    std::ifstream infile(file.c_str(),std::ios::in);
  //将文件流对象与文件连接起来
    assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

    string s;
    std::vector<string> vec;
    while(getline(infile,s))
    {
        vec.push_back(s);
    }
    infile.close();             //关闭文件输入流
    return vec;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "label_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ros::Rate loop_rate(10);
    std::string load_file;
    std::string label_index;
    private_nh.param("load_file",load_file,std::string("src/label_tool/data/ROC/train/train_file.txt"));
    private_nh.param("label_index",label_index,std::string("car"));
    std::vector<string> filenameVec;
    filenameVec = readTxt(load_file);
filenameVec.clear();
string ff = "src/label_tool/data/ROC/test/testpc.txt";
filenameVec.push_back(ff);
    for(int i = 0; i<filenameVec.size();i++)
    {
        TrackManager tm(filenameVec[i]);
        std::cout << "Loaded " << tm.tracks_.size() << " tracks." << std::endl;
        std::cout << std::endl;
        pcl::PointCloud<pointType>::Ptr pointCloudSeg(new pcl::PointCloud<pointType>);
        FeatureExtract fe;

        for(int tck = 0; tck < tm.tracks_.size(); tck ++)
        {
            int label = 0;
            if(tm.tracks_[tck]->label_ == label_index)
            {
                label = 1;
            }
            else
            {
                label = -1;
            }
            for(int sg = 0; sg <tm.tracks_[tck]->segments_.size(); sg = sg+1)
            {
                *pointCloudSeg = *tm.tracks_[tck]->segments_[sg]->cloud_;

                if(label != 0)
                {
                    fe.featureExtract(pointCloudSeg,label);
                }
                if(pointCloudSeg->points.size() != 0)
                      pointCloudSeg->clear();
            }
            if(!ros::ok())
                break;
        }

    }
  return 0;
  }
