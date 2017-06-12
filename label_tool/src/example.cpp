#include"track_label.h"
#include<iomanip>
#include "std_msgs/Int16.h"
#include"std_msgs/Int16MultiArray.h"
#include"std_msgs/String.h"

int control = 0;
std::string label;
int label_mark = 0;
void controlCallback(const std_msgs::Int16& msg)
{
    control = msg.data;
    ROS_INFO("%d",control);
}
void labelCallback(const std_msgs::String::ConstPtr& msg)
{
    label=msg->data.c_str();
    label_mark = 1;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "label_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ros::Publisher pub = node.advertise<sensor_msgs::PointCloud2>("pointcloud",1);
    ros::Publisher track_num_pub = node.advertise<std_msgs::Int16MultiArray>("track_num",1);//0是当前track  1是总track
    ros::Publisher seg_num_pub = node.advertise<std_msgs::Int16MultiArray>("seg_num",1);//0是当前seg  1是总seg
    ros::Subscriber control_sub  = node.subscribe("control_signal",1,controlCallback);
    ros::Subscriber label_sub = node.subscribe("label_signal",1,labelCallback);

    ros::Rate loop_rate(10);
    std::string filename_;// = "/home/fan/trackfile.txt";
    std::string filename_wr;// = "/home/fan/labeled.txt";
    std::string unlabel_file ;//= "/home/fan/unlabeled.txt"


    private_nh.param("load_file", filename_ , std::string("/home/fan/trackfile.txt"));
    private_nh.param("labeled_file", filename_wr , std::string("/home/fan/changethename1.txt"));
    private_nh.param("unlabel_file", unlabel_file , std::string("/home/fan/changethename2.txt"));




    TrackManager tm(filename_);
    std::cout << "Loaded " << tm.tracks_.size() << " tracks." << std::endl;
    std::cout << std::endl;

    int track_count = 0;
    int seg_count = 0;

    int pub123 = 0;

    while(ros::ok())
    {
        if(control)
        {
            pub123 =1;
        }
        if(control == 1)//next track
        {
            control=0;
            track_count++;
            if(track_count < tm.tracks_.size())
            {
                seg_count = 0;
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_),outpc  );
                pub.publish(outpc);
            }else
            {
                track_count = (tm.tracks_.size()-1);
            }
			control = 6;
        }else if(control == 2)//next seg
        {
            control=0;
            seg_count++;
            if(seg_count < tm.tracks_[track_count]->segments_.size())
            {
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_),outpc  );
                pub.publish(outpc);
            }else
            {
                seg_count = (tm.tracks_[track_count]->segments_.size())-1;
            }
        }else if(control == 3)// last seg
        {
            control=0;
            seg_count--;
            if(seg_count >=0)
            {
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_) ,outpc  );
                pub.publish(outpc);
            }else
            {
                seg_count = 0;
            }
        }else if(control == 4)//label
        {
            control=0;
            //save///////////////////////////
            //std::cout<< label << std::endl;
            tm.tracks_[track_count]->label_ = label;
            track_count++;
            if(track_count < tm.tracks_.size())
            {
                control=0;
                seg_count = 0;
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_) ,outpc  );
                pub.publish(outpc);
                control = 6;
            }else
            {
                track_count = tm.tracks_.size();
                control = 8;
            }

        }else if(control == 5)//delete
        {
            control = 0;
            std::vector<boost::shared_ptr<Segment> >::iterator it = tm.tracks_[track_count]->segments_.begin();
            it+=seg_count;
            tm.tracks_[track_count]->segments_.erase(it);
            seg_count--;
            if(seg_count < 0)
                seg_count = 0;
            if(seg_count > tm.tracks_[track_count]->segments_.size())
            {
                seg_count = tm.tracks_[track_count]->segments_.size();
            }
            std::vector<int> a ;
            if(tm.tracks_[track_count]->segments_.empty())
            {
                control = 1;
                continue;
            }

            sensor_msgs::PointCloud2 outpc;
            pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_.at(seg_count)->cloud_) ,outpc  );
            pub.publish(outpc);

        }else if(control == 6)//play seg
        {
             control = 6;
            seg_count++;
            if(seg_count < tm.tracks_[track_count]->segments_.size())
            {
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_),outpc  );
                pub.publish(outpc);
            }else
            {
                seg_count = (tm.tracks_[track_count]->segments_.size())-1;
            }
        }else if(control == 7)//replay seg
        {
            control = 7;
            seg_count--;
            if(seg_count < 0) seg_count = 0;
            if(seg_count < tm.tracks_[track_count]->segments_.size())
            {
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_),outpc  );
                pub.publish(outpc);
            }else
            {
                seg_count = (tm.tracks_[track_count]->segments_.size())-1;
            }
        }else if(control == 8)//save  track
        {
            for(int tk = track_count; tk < tm.tracks_.size();++tk)
            {
                //tm.tracks_[track_count]->label_ = label;
                tm.tracks_[tk]->serialize(unlabel_file);
                std::cout<<"Saving "<< tk <<"track .unLabeled " << tm.tracks_.size() << std::endl;
            }
            for(int tk = 0; tk < track_count;++tk)
            {
                //tm.tracks_[track_count]->label_ = label;
                tm.tracks_[tk]->serialize(filename_wr);
                std::cout<<"Saving "<< tk <<"track .labeled " << tm.tracks_.size() << std::endl;
            }
            std::cout<< "Done~" << std::endl;
            break;
        }else if(control == 9)//last track
        {
            control=0;
            track_count--;
            if(track_count >= 0)
            {
                seg_count = 0;
                sensor_msgs::PointCloud2 outpc;
                pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_[seg_count]->cloud_),outpc  );
                pub.publish(outpc);
            }else
            {
                track_count = 0;
            }
            control = 6;
        }else if(control == 10)//delete track
        {
            control = 0;
            std::vector<boost::shared_ptr<Track> >::iterator it = tm.tracks_.begin();
            it+=track_count;
            tm.tracks_.erase(it);
            if(track_count < 0)
                track_count = 0;
            if(track_count > (tm.tracks_.size()-1))
            {
                track_count = tm.tracks_.size()-1;
            }
            if(tm.tracks_.empty())
            {
                break;
            }
            seg_count = 0;
            sensor_msgs::PointCloud2 outpc;
            pcl::toROSMsg(    *(tm.tracks_[track_count]->segments_.at(seg_count)->cloud_) ,outpc  );
            pub.publish(outpc);
            control = 6;
        }



        std_msgs::Int16MultiArray track_msg;
        std_msgs::Int16MultiArray seg_msg;

        if(pub123)
        {
            pub123 = 0;
            track_msg.data.push_back(track_count);
            track_msg.data.push_back(tm.tracks_.size());
            track_num_pub.publish(track_msg);
            seg_msg.data.push_back(seg_count);
            seg_msg.data.push_back(tm.tracks_[track_count]->segments_.size());
            seg_num_pub.publish(seg_msg);
        }



        loop_rate.sleep();
        ros::spinOnce();

    }




}

