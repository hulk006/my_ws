#ifndef DRAWING_H
#define DRAWING_H
#include "track.h"
namespace Robosense
{
class Drawing
{
public:
  Drawing();
  ~Drawing();

/* @brief drawing the tracking and point cloud
   @param[in] tracking, the tracking object.
   @param[in] current_pointcloud, the whole pointcloud.
*/
  void drawInRviz(ros::Publisher& vel_pub,ros::Publisher& pc_pub,const Robosense::Tracking& tracking, pcl::PointCloud<pcl::PointXYZI> &current_pointcloud);
private:
  void drawRect(visualization_msgs::Marker &cuboid_marker, const Robosense::Target target);
  void predictVehicleRect(rect_info &target_rectangle, Target head_target, float deta_x);
  void pushBackCubePoints(visualization_msgs::Marker &cuboid_marker, geometry_msgs::Point coboid[]);
  void selfTurning(const Robosense::Tracking& tracking);
  const float PI_;
  bool self_turning_;
  float v_threhold_;
};
}//end Robosense namespace
#endif
