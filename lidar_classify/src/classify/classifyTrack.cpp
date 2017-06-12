#include "classifyTrack.h"
#include "drawing.h"
using std::string;
ClassifyTrack::ClassifyTrack(ros::NodeHandle node, ros::NodeHandle private_nh,
                             std::vector<string> modelRangeFiles)
{
    seg_sub = node.subscribe("clusters", 10, &ClassifyTrack::classifyTrackCallback,(ClassifyTrack*)this);
    vel_pub = node.advertise<visualization_msgs::MarkerArray>("velocity",10);
    pc_pub = node.advertise<sensor_msgs::PointCloud2>("pc_cloud",10);
    mcf.setModelRange(modelRangeFiles[0],modelRangeFiles[1]);
    MM.setVelFilePath(modelRangeFiles[2], 0);
    // MM.setVelFilePath(modelRangeFiles[2], modelRangeFiles[3],0);

}
void ClassifyTrack::classifyTrackCallback(const lidar_msg::clusterArray& msg)
{
   //cout<<"msg_header:"<<msg.header.frame_id<<std::endl;
    clock_t start = clock();
    std::vector<pcl::PointCloud<pcl::PointXYZI> > seg_vec;
    pcl::PointCloud<pcl::PointXYZI> pCloud;
    std::vector<Robosense::SegMsg> segmsg2_0;
    MM.convertMsgForTracking(msg,pCloud,segmsg2_0);
    MM.trackSegMain(segmsg2_0);
    classifySeg(MM);
    Robosense::Drawing drawing;
    drawing.drawInRviz(vel_pub,pc_pub,MM, pCloud);
//    generateSegfile(MM);
    clock_t end1 = clock();
    std::cout << "the cost1 time is: " << (double)(end1 - start)/1000 << " ms" << std::endl;
}

void ClassifyTrack::classifySeg(Robosense::Tracking& MM)
{
    for(int i = 0; i<MM.target_current_.size();i++)
    {
        std::vector<float> tmprob;
        int label = mcf.objectClassify(MM.target_current_[i].pointcloud_.makeShared(),tmprob);

        int matchIndex = MM.target_current_[i].match_location_;
        if(matchIndex != -1)
        {
            int frame = MM.tracks_vector_[matchIndex].target_buffer_.size();
            if(MM.classify_flag_ == true)
            {
                label = 2;
            }
            else if(MM.tracks_vector_[matchIndex].target_buffer_[frame - 1].classify_ == 2)
            {
                label = 2;
                MM.classify_flag_ = true;

            }


        }
        MM.target_current_[i].classify_ = label;
    }

}

void ClassifyTrack::classifyBayesFilter(Robosense::Tracking& MM)
{
    for(int i = 0; i<MM.target_current_.size();i++)
    {
        int label = 0;
        std::vector<float> tmprob;
        int matchIndex = MM.target_current_[i].match_location_;
        label = mcf.objectClassify(MM.target_current_[i].pointcloud_.makeShared(),tmprob);
        std::vector<float>::iterator biggest = std::max_element(tmprob.begin(), tmprob.end());
        float prob = *biggest;
        //**************
//        for(int t = 0;t<tmprob.size();t++)
//        {
//            std::cout<<tmprob[t]<<" ";
//        }
//        std::cout<<prob<<std::endl;
        //*****************
        if(matchIndex != -1)
        {
            if((MM.tracks_vector_[matchIndex].target_buffer_.size() > 10)&&(prob < 0.9))
            {
                std::vector<float> probfilter(tmprob.size(),0);
                int num_frame = 0;
                for(int kk = 1; kk<MM.tracks_vector_[matchIndex].target_buffer_.size()-1;kk++)
                {
                    num_frame++;
                    for(int jj = 0; jj < MM.tracks_vector_[matchIndex].target_buffer_[kk].classify_probability_.size();jj++)
                    {
                        probfilter[jj] += MM.tracks_vector_[matchIndex].target_buffer_[kk].classify_probability_[jj];
                    }
                }
                for(int jj = 0; jj<probfilter.size();jj++)
                {
                    probfilter[jj] /= num_frame;
                }

                label = labelByProb(probfilter);
                tmprob = probfilter;

            }
            MM.tracks_vector_[matchIndex].target_buffer_[MM.tracks_vector_[matchIndex].target_buffer_.size() - 1].classify_ = label;
            MM.tracks_vector_[matchIndex].target_buffer_[MM.tracks_vector_[matchIndex].target_buffer_.size() - 1].classify_probability_ = tmprob;
        }
        MM.target_current_[i].classify_ = label;
        MM.target_current_[i].classify_probability_ = tmprob;
    }

}

int ClassifyTrack::labelByProb(std::vector<float>& prob)
{
    int maxclass = -1;
    int maxprob = -1;
    for(int i = 0; i<prob.size();i++)
    {
        if(prob[i] > maxprob)
        {
            maxprob = prob[i];
            maxclass = i;
        }
    }
    return maxclass;

}

void ClassifyTrack::generateSegfile(Robosense::Tracking& MM)
{
  std::ofstream segFile;
  std::string segFilePath = ros::package::getPath("lidar_classify");
  segFilePath = segFilePath + "/../data/segfile.txt";
  segFile.open(segFilePath.c_str(), std::ios::app);
  segFile << "Track"      << std::endl;
  segFile << "label"      << std::endl;
  segFile << "unlabled"   << std::endl;
  segFile << "num_frames" << std::endl;
  segFile << MM.target_current_.size() << std::endl;

  for (int i = 0; i < MM.target_current_.size(); ++i)
  {
    segFile << "seg"          << std::endl;
    segFile << "segPointsNum" << std::endl;
    segFile << MM.target_current_[i].pointcloud_.size() << std::endl;
    for(int ii = 0; ii < MM.target_current_[i].pointcloud_.size(); ++ii)
    {
    pcl::PointXYZI point = MM.target_current_[i].pointcloud_[ii];
    segFile.write((char*)&point.x, sizeof(double));
    segFile.write((char*)&point.y, sizeof(double));
    segFile.write((char*)&point.z, sizeof(double));
    segFile.write((char*)&point.intensity, sizeof(double));
    segFile << std::endl;
    }

  }
  segFile.close();
}

void ClassifyTrack::initMarker(visualization_msgs::Marker& marker)
{
    marker.header.frame_id = "map2";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shape_rect";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 1.1;
    marker.scale.z = 1.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}
ClassifyTrack::~ClassifyTrack()
{

}
