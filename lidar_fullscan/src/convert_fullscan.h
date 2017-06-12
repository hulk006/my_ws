#ifndef CONVERT_FULLSCAN_
#define CONVERT_FULLSCAN_

#include <ros/ros.h>
#include <lidar_msg/VelodynePic.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>
#include  "myparam.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "modify_driver.h"
int fullscan = 0;
ros::Publisher pic_output;
ros::Publisher pc_output;
lidar_msg::VelodynePic pic;
int correct[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};
float PI=3.1415926535897;
void init_setup()
{
    pic.col=0;
    pic.distancenum=0;
    pic.intensitynum=0;
    pic.azimuth.resize(POINT_PER_CIRCLE_);
    pic.distance.resize(DATA_NUMBER_PER_SCAN);//288000
    pic.intensity.resize(DATA_NUMBER_PER_SCAN);//288000
    pic.azimuthforeachP.resize(DATA_NUMBER_PER_SCAN);
}


void unpack(const velodyne_msgs::VelodynePacket &pkt,const std::string frame_id)
{
    float azimuth;
    float intensity;
    float azimuth_diff;
    float last_azimuth_diff;
    float azimuth_corrected_f;
    int azimuth_corrected;
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    for (int block = 0; block < BLOCKS_PER_PACKET; block++ , ++pic.col)
    {
        if(UPPER_BANK != raw->blocks[block].header)
        {
            ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                                        << block << " header value is "
                                                        << raw->blocks[block].header);
            return ;
        }
        azimuth = (float)(raw->blocks[block].rotation);
        if (block < (BLOCKS_PER_PACKET-1))//12
        {
            azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);//40
            last_azimuth_diff = azimuth_diff;
        }else
        {
            azimuth_diff = last_azimuth_diff;
        }
        //if(((pic.col>100)&&(abs(azimuth-pic.azimuth[0])<100))||(pic.col==(POINT_PER_CIRCLE_-2)))
        if(((abs(azimuth - 0) < 50)&&(pic.col>100))||(pic.col==(POINT_PER_CIRCLE_-2)))//0附近为起点
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
            pointcloud->height = 16;
            pointcloud->width = 2*pic.col;
            pointcloud->is_dense = true;
            pointcloud->resize(pointcloud->height * pointcloud->width);
            pointcloud->header.frame_id = frame_id;
            for (int block = 0; block < pic.col; block++)
            {
                for (int firing = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
                {
                    for (int laserid = 0; laserid < VLP16_SCANS_PER_FIRING; laserid++)
                    {
                        double dis = pic.distance[block*32 + correct[laserid] + 16*firing];
                        double arg_horiz = pic.azimuthforeachP[block*32 + correct[laserid] + 16*firing] /18000*PI;
                        double intensity = pic.intensity[block*32 + correct[laserid] + 16*firing];
                        double arg_vert = VERT_ANGLE2[laserid];

                        pcl::PointXYZI point;
                        point.x = dis * cos(arg_vert) * sin(arg_horiz);
                        point.y = dis * cos(arg_vert) * cos(arg_horiz);
                        point.z = dis * sin(arg_vert);
                        point.intensity = intensity;
                        pointcloud->at(2*block + firing, laserid) = point;
                    }
                }
            }
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*pointcloud, out);
            out.header.stamp = pkt.stamp;
            pc_output.publish(out);

            pic_output.publish(pic);
            init_setup();
            pic.header.stamp = pkt.stamp;
            //ROS_INFO("%d",pic.header.stamp.sec);
            fullscan = 1;
        }
        pic.azimuth[pic.col]=azimuth;

        for(int firing = 0 ,k = 0;firing < VLP16_FIRINGS_PER_BLOCK; firing++)//2
        {
            for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE)//16   3
            {   
                azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);// 2.304f    55.296f   110.592f
                //修正项最大是0.812*40
                azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;//convert to integral value...
                pic.azimuthforeachP[pic.col*32+k/3]=azimuth_corrected;
                union two_bytes tmp;
                tmp.bytes[0] = raw->blocks[block].data[k];
                tmp.bytes[1] = raw->blocks[block].data[k+1];
                float distance = tmp.uint * DISTANCE_RESOLUTION;

                pic.distance[pic.col*32+k/3] = distance;
                pic.distancenum++;

                intensity = raw->blocks[block].data[k+2];
                float min_intensity=0, max_intensity = 255;
                intensity = (intensity < min_intensity) ? min_intensity : intensity;
                intensity = (intensity > max_intensity) ? max_intensity : intensity;

                pic.intensity[pic.col*32+k/3] = intensity;
                pic.intensitynum++;

            }
        }

    }
}
#endif


