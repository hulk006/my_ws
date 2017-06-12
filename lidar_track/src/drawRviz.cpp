//
// Created by mjj on 17-5-27.
//

#include "drawRviz.h"

drawRviz::drawRviz(){
    cv::generateColors(_colors,pubNum);
}

void drawRviz::drawTrack(const ros::Publisher& pub_trackInfo, const ros::Publisher& pub_pcloud, const std_msgs::Header _header,
                         const std::vector<precision_tracking::Tracker>& trackers){

    visualization_msgs::MarkerArrayPtr markersArray(new visualization_msgs::MarkerArray);
    visualization_msgs::Marker markerTexts;
    visualization_msgs::Marker markerArraws;
    visualization_msgs::Marker markerRect;
    int markerID = 0;
    int trackerSize = trackers.size();
    if (trackerSize > pubNum){
        pubNum = trackerSize;
        cv::generateColors(_colors,pubNum);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < trackers.size(); ++i) {
        if (!trackers[i].isCurFrame)continue;
        for (int j = 0; j < trackers[i].previousModel_->size(); ++j) {
            pcl::PointXYZI tmpI = trackers[i].previousModel_->points[j];
            pcl::PointXYZRGB tmpRGB;
            tmpRGB.x = tmpI.x;tmpRGB.y = tmpI.y;tmpRGB.z = tmpI.z;
            tmpRGB.r = _colors[i][0];tmpRGB.g = _colors[i][1];tmpRGB.b = _colors[i][2];
            cur_cloud_ptr->push_back(tmpRGB);
        }

        //velocity
        float abs_velocity = std::sqrt(std::pow(trackers[i].cur_x_abs_velocity,2) + std::pow(trackers[i].cur_y_abs_velocity,2));
        abs_velocity *= 3.6;
        abs_velocity = (int)abs_velocity;

        //----------------bounding_box---------------
        Eigen::MatrixXf boudingTrack = *(trackers[i].bounding_box_);
        //----------------marker track info----------
        markerTexts.id = markerID;
        markerID++;
        markerTexts.ns = "track_info";
        markerTexts.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        markerTexts.action = visualization_msgs::Marker::ADD;
        markerTexts.pose.position.x = boudingTrack(0,0) + (boudingTrack(0,1) - boudingTrack(0,0))/2.;
        markerTexts.pose.position.y = boudingTrack(1,0) + (boudingTrack(1,1) - boudingTrack(1,0))/2.;
        markerTexts.pose.position.z = 1.;
        markerTexts.scale.x = 0.8;
        markerTexts.scale.y = 0.8;
        markerTexts.scale.z = 0.8;
        markerTexts.color.r = 0.;
        markerTexts.color.g = 1.;
        markerTexts.color.b = 0.;
        markerTexts.color.a = 1.;
        markerTexts.text = std::string(num2str(abs_velocity)) + std::string(" km/h");
        markerTexts.header = _header;
        markersArray->markers.push_back(markerTexts);

        //-----------------rect---------------------
        markerRect.header = _header;
        markerRect.ns = "track_rect";
        markerRect.type = visualization_msgs::Marker::LINE_LIST;
        markerRect.action = visualization_msgs::Marker::ADD;
        markerRect.id = markerID;
        markerRect.pose.position.x = 0.;
        markerRect.pose.position.y = 0.;
        markerRect.pose.position.z = 0.;
        markerRect.scale.x = 0.05;
        markerRect.scale.y = 0.05;
        markerRect.scale.z = 0.05;
        markerRect.color.r = 0.;
        markerRect.color.g = 1.;
        markerRect.color.b = 1.;
        markerRect.color.a = 1.;
        drawRect(markerRect,trackers[i]);
        markersArray->markers.push_back(markerRect);
    }

    for (int i = markerID; i < pubNum; ++i) {
        markerTexts.id = markerID;
        markerID++;
        markerTexts.color.a = 0.;
        markersArray->markers.push_back(markerTexts);
        markerRect.id = markerID;
        markerRect.color.a = 0.;
        markersArray->markers.push_back(markerRect);

    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cur_cloud_ptr, cloud_msg);
    cloud_msg.header=_header;
    pub_pcloud.publish(cloud_msg);
    pub_trackInfo.publish(markersArray);
}

void drawRviz::drawRect(visualization_msgs::Marker &cuboid_marker, const precision_tracking::Tracker& singleTracker){

    pcl::PointCloud<pcl::PointXYZI>::Ptr single_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    *single_cloud_ptr = *(singleTracker.previousModel_);
    pcl::PointXYZI maxPt,minPt;
    pcl::getMinMax3D(*single_cloud_ptr,minPt,maxPt);
    // draw the cuboid
    float x_theta = singleTracker.cur_angle_in_own_xaxi_;
    float xmin = minPt.x;
    float xmax = maxPt.x;
    float ymin = minPt.y;
    float ymax = maxPt.y;
    float zmin = minPt.z;
    float zmax = maxPt.z;
    geometry_msgs::Point cuboid[10];
    // default cuboid
    cuboid[4].x = xmin;  cuboid[4].y = ymin;  cuboid[4].z = zmin;
    cuboid[5].x = xmax;  cuboid[5].y = ymin;  cuboid[5].z = zmin;
    cuboid[6].x = xmax;  cuboid[6].y = ymax;  cuboid[6].z = zmin;
    cuboid[7].x = xmin;  cuboid[7].y = ymax;  cuboid[7].z = zmin;
    cuboid[0].x = xmin;  cuboid[0].y = ymin;  cuboid[0].z = zmax;
    cuboid[1].x = xmax;  cuboid[1].y = ymin;  cuboid[1].z = zmax;
    cuboid[2].x = xmax;  cuboid[2].y = ymax;  cuboid[2].z = zmax;
    cuboid[3].x = xmin;  cuboid[3].y = ymax;  cuboid[3].z = zmax;

    cv::Point2f center;
    center.x = (xmax + xmin) / 2;
    center.y = (ymax + ymin) / 2;
    pushBackCubePoints(cuboid_marker, cuboid);
}

void drawRviz::pushBackCubePoints(visualization_msgs::Marker &cuboid_marker, geometry_msgs::Point cuboid[])
{
    // horizontal low points for lines
    cuboid_marker.points.push_back(cuboid[0]);
    cuboid_marker.points.push_back(cuboid[1]);
    cuboid_marker.points.push_back(cuboid[1]);
    cuboid_marker.points.push_back(cuboid[2]);
    cuboid_marker.points.push_back(cuboid[2]);
    cuboid_marker.points.push_back(cuboid[3]);
    cuboid_marker.points.push_back(cuboid[3]);
    cuboid_marker.points.push_back(cuboid[0]);
    // horizontal high points for lines
    cuboid_marker.points.push_back(cuboid[4]);
    cuboid_marker.points.push_back(cuboid[5]);
    cuboid_marker.points.push_back(cuboid[5]);
    cuboid_marker.points.push_back(cuboid[6]);
    cuboid_marker.points.push_back(cuboid[6]);
    cuboid_marker.points.push_back(cuboid[7]);
    cuboid_marker.points.push_back(cuboid[7]);
    cuboid_marker.points.push_back(cuboid[4]);
    // vertical points for lines
    cuboid_marker.points.push_back(cuboid[0]);
    cuboid_marker.points.push_back(cuboid[4]);
    cuboid_marker.points.push_back(cuboid[1]);
    cuboid_marker.points.push_back(cuboid[5]);
    cuboid_marker.points.push_back(cuboid[2]);
    cuboid_marker.points.push_back(cuboid[6]);
    cuboid_marker.points.push_back(cuboid[3]);
    cuboid_marker.points.push_back(cuboid[7]);
}