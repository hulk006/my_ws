//
// Created by mjj on 17-5-3.
//

#include <ros/ros.h>
#include "lidar_msg/terrian.h"
#include <fstream>
#include <iostream>
//#include "velodyne_driver_mod/roi.h"

using namespace std;
vector<vector<string> > roiPt;
double roiStamp = 0.;
double terrianStamp = 0.;
int indexStamp = 0;
ros::Publisher pub_roiTerrian;

void terrianCallback(const lidar_msg::terrian msg)
{
    lidar_msg::terrian roiMsg = msg;
    roiMsg.header.frame_id = msg.header.frame_id;
    terrianStamp = msg.header.stamp.toSec();
    int tmp;
    for (; indexStamp < roiPt.size() - 1; ++indexStamp) {
        long long int cur = atoll(roiPt[indexStamp][1].c_str());
        long long int next = atoll(roiPt[indexStamp + 1][1].c_str());
        roiStamp = cur / 1000000.;
        double nextStamp = next / 1000000.;
        if (abs(terrianStamp - roiStamp) < 0.00005)
        {
            for (int i = 2; i < roiPt[indexStamp].size() - 2; ) {
                float tmpX = atof(roiPt[indexStamp][i].c_str());
                tmpX = tmpX < 0 ? tmpX + 1.5:tmpX - 1.5;
                float tmpY = atof(roiPt[indexStamp][i + 1].c_str());
                //tmpY = tmpY < 0? tmpY + 1.5 : tmpY - 1.5;
                roiMsg.ROIptx.push_back(tmpX);
                roiMsg.ROIpty.push_back(tmpY);

                i = i + 3;
            }
            tmp = indexStamp;
            break;
        }

    }

    indexStamp = tmp - 5 < 0? 0:tmp - 5;

    cout<<"the "<<roiPt[indexStamp][0]<<"frame: ";
    for (int j = 0; j < roiMsg.ROIptx.size(); ++j) {
        cout<<roiMsg.ROIptx[j]<<" "<<roiMsg.ROIpty[j]<<" ";
    }
    cout<<endl;
    pub_roiTerrian.publish(roiMsg);
}

string& replace_all_distinct(string&   str, const   string&   old_value, const   string&   new_value)
{
    for (string::size_type pos(0); pos != string::npos; pos += new_value.length())   {
        if ((pos = str.find(old_value, pos)) != string::npos)
            str.replace(pos, old_value.length(), new_value);
        else   break;
    }
    return   str;
}

void readFromTxt(string path);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "terrianROI");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    string topic = private_nh.param("topic",string("terrian"));

    ros::Subscriber sub=node.subscribe(topic , 10 , terrianCallback);
    pub_roiTerrian = node.advertise<lidar_msg::terrian>("terrianROI",10);

    string txtPath = private_nh.param("txtPath",string("/home/mjj/data/lidar/roi.txt"));
    readFromTxt(txtPath);
    int a = 0;
    vector<string> ts = roiPt[3162];
    // handle callbacks until shut down
    ros::spin();

    return 0;
}

void readFromTxt(string path)
{
    ifstream ifile;
    ifile.open(path.c_str());
    assert(ifile.is_open());

    string str;
    while (!ifile.eof())
    {
        getline(ifile, str);
        //cout << str<< endl;
        replace_all_distinct(str,"("," ");
        replace_all_distinct(str,")"," ");
        replace_all_distinct(str," - 100"," ");
        replace_all_distinct(str,","," ");
        istringstream infile(str);

        string c;
        vector<string> tmpString;
        tmpString.clear();
        while (!infile.eof())
        {
            infile >> c;
            tmpString.push_back(c);
        }
        roiPt.push_back(tmpString);
    }
}
