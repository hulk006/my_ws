#ifndef TRACK_LABEL_H
#define TRACK_LABEL_H

#include<ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

class Segment
{
public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

    bool deserialize(std::istream& istrm);
};

class Track
{
public:
    std::string label_;
    std::vector<boost::shared_ptr<Segment> > segments_;

    bool deserialize(std::istream& istrm);
    void serialize(std::string & filename);

};

class TrackManager
{
public:
    std::vector<boost::shared_ptr<Track> > tracks_;

    TrackManager(const std::string& filename );

    bool deserialize(std::istream& istrm);


};














#endif
