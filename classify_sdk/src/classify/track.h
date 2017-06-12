#ifndef TRACK_H
#define TRACK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/icp_nl.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <map>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>

#include "boxEstimater.h"
#include <lidar_msg/clusterArray.h>

namespace Robosense
{

/* @brief Classification item.
   Init for the Tracking class initialize. Others are self-describes.
*/
enum TargetClassify
{
  Init = 0,
  Car,
  Truck,
  Ped,
  Unknow
};

/* @brief The cuboid information of the segment pointcloud.
 */
typedef struct SegResult
{
  float max_x;
  float max_y;
  float max_z;
  float min_x;
  float min_y;
  float min_z;
} SegResult;

/* @brief The segmentation message.
 */
typedef struct SegMsg
{
  SegResult seg_result;
  float rect_length;
  float rect_width;
  float rect_label; //!< for the classification label
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
} SegMsg;

/* @brief The Target Class contain all the target information
 */
class Target
{
public:
  Target();
  ~Target();
  Target(const Target& target);
  Target& operator=(const Target& target);
  void initTarget(const Target &target);

  unsigned int vanish_time_; // target vanish time, couts for delete the vanish target
  int match_location_; // In head_target, it points to the tracking location in last frame in target_vector.
                       // In target vector, it points corresponding location to in head_target.
  int pnumber_; // The segment cloud point number.
  int state_;   // The tracking state. 0 represents new target; 1 represents the target has been tracked; 2 represents the target is vanished.
  int istracking_; // 0 represents new; 1 represents having tracking, used in head_target and last target in target_vector.
  int flag_match_; // The flag represents the target is matched or not.
  int match_id_; // The id of the tracking target.
  int classify_; // The classification of target. @see TargetClassify

  float distance_; // The distace between the center of segementation and the lidar.
  float width_; // The width of the segementation rectangle.
  float length_;// The length of the segementation rectangle.
  float v_relative_; // The target's velocity relative to the lidar.
  float v_angle_relative_;  // The relative clockwise angle relative to the x axis.
  float v_angle_in_own_xaxis_;
  float v_angle_absolute_; // The global velocity of the target.

  float vx_relative_; // The relative velocity in x axis between every frame (m/s).
  float vy_relative_; // The relative velocity in y axis (m/s).
  float vx_absolute_;//The absolute velocity in map x aixs  (m/s)
  float vy_absolute_;//The absolute velocity in map y aixs  (m/s)

  float v_mean_absolute_; // The abosolute mean velocity in one second (m/s).
  float vx_mean_relative_; // The relative mean velocity in the x axis, to detect the target is turning or not.
  std::vector<float> classify_probability_; // The probability of the classification.

  Robosense::rect_info rect_info_; // The rectangle information contain the main direction for drawing.
  cv::Point2f center_; // The center of the rectangle
  Robosense::SegResult cube_; // The target is regarded as cuboid

  pcl::PointCloud<pcl::PointXYZI> pointcloud_;
  cv::Point rectangle_[4]; // Store the rectangle four points's coodination.
};

class TargetKalman
{
public:
  TargetKalman();
  ~TargetKalman();

  boost::circular_buffer<Robosense::Target> target_buffer_; // Every target and the corresponding tracked target stored in the circular_buffer, new target using another circular_buffer.

  CvKalman* cvkalman_; // The struct point of CvKalman, for the kalman tracking.
  CvMat* statematrix_; // The state [x; vx; y; vy]
  CvMat* measurement_; // The measurement. [x; vx; y; vy]

  /* @brief Initialize the Kalman.
     Assignment of the state_post, transition_matrix, measurement_matrix, process_noise_cov, error_cov_post, measurement_noise_cov.
     @param[in] x the x coordinate of the target.
     @param[in] y the y coordinate of the target.
     @param[in] t the update time, the lidar update time.
  */
  void initKalman(const float x, const float y, const float t);

  /* @brief Kalman filter
     @param[in] last_frame the last state.
     @param[in] current_frame the current measurement.
  */
  void getTargetpoint(const float *last_frame, const float *curent_frame);

};


/* @brief Tracking Class
   The tracking class define the circular_buffer for every target, tracking the target using kalman filter, include the geometry features, include the number of the point cloud, the ratio of the width and length e.t.c
*/
class Tracking
{
public:
  Tracking();
  ~Tracking();

  /* @brief Set the offine velocity file.
     @param[in] file the offline velocity file.
     @param[in] mode 0, obd velocity file; 1 the mapping velocity file.
  */
  void setVelFilePath(const std::string& vel_file, int mode);
  void setVelFilePath(const std::string& obd_file,const std::string& mapping_file, int mode);

  /* @brief convert clusterArray type to pointcloud and SegMsg for tracking.
     @param[in] msg the clusterArray type for the classify result.
     @param[out] current_pointcloud the all pointcloud.
     @param[out] tracking_msg the tracking message for later tracking.
  */
  void convertMsgForTracking(const lidar_msg::clusterArray &msg, pcl::PointCloud<pcl::PointXYZI> &current_pointcloud, std::vector<SegMsg> &tracking_msg);

  /* @brief The main function for tracking.
     Include the initilization, tracking, and release.
     @param[in] seg_msg The segmentation message for tracking.
     @param[in] current_pointcloud The segment pointcloud.
  */
  void trackSegMain(const std::vector<Robosense::SegMsg> &seg_msg);
  std::vector<Robosense::TargetKalman> tracks_vector_;
  std::vector<Target> target_current_;// The current target
  float self_velocity_; // The self vehicle abosolute velocity (m/s).
  bool classify_flag_;//track classify flag
  boost::circular_buffer<float > yaw_;

private:
  /* @brief The initialization function for tracking.
     @param[in] head_target The current target
     @param[in] current_pointcloud the current target pointcloud.
  */
  void initForTracking(const std::vector<Robosense::SegMsg> &seg_msg);

  /* @brief Calculate the transformation matrix.
     @param[in] self_v The absolute self vehicle velocity.
  */
  void calculateTransformation(const float &self_v);

  /* @brief Delete the vanished target.
   */
  void deleteVanishTarget(void);

  /* @brief Add the new target to the target_vector.
     @param[in] head_target current new targets have not been tracked.
  */
  void addCurrentTarget(std::vector<Target> &head_target);

  /* @brief The first tracking using kalman and some constrains.
     @param[in] head_target current targets to be tracked.
  */
  void trackTarget(std::vector<Target> &head_target);

  /* @brief The function do the kalman filter called by trackTarget.
     @param[in] current_target, current target which is macthed to ip
     @param[in] ip the corresponding target in the track_vector.
     @param[in] vel the velocity of the match target in current frame coordination.
  */
  void doMatch( Target &current_target, std::vector<Robosense::TargetKalman>::iterator &ip,Target &temp_target, cv::Point2f v);

  /*@brief Judget the state of the targets have not match in the tracks_vector.
   */
  void stateJudgement(void);

  /* @brief The tracking for vanish targets.
   */
  void trackVanishTarget(void);

  /*@brief smooth filter for the velocity and angle.
    @param[in] head_target the current target.
  */
  void smoothFilter(std::vector<Target> &head_target);

  /* @brief The tracking for low velocity targets.
   */
  void trackLowVelTarget(void);

  /* @brief Release the resource of the Targets, current empty.
   */
  void releaseTarget(void);

  /* @brief Read the self velocity of the vehicle from the file.
   */
  void readSelfVelocity(void);
  void readSelfVelocityObd(void);
  void readSelfVelocityMap(void);

  /* @brief Calculate the target velocity.
     @param[in] target, the input target.
     @ret  The abosolute velocity of the target.
  */
  float caculateVelocity(Target & target);

  /* @brief Get the min max x, y, z coordination of pointcloud.
     @param[in] pointcloud_vec the pointcloud.
     @ret the min max coordination.
  */
  Robosense::SegResult getMinMaxSeg(pcl::PointCloud<pcl::PointXYZI> vec);

  /* @brief Get the self_velocity online.
     @param[in] msg the message with abosolute velocity online.
  */
  void getVelocityCallback(const std_msgs::Float32MultiArray &msg);

  /* @brief Get the rectangle infomation of the target, especially the main direction.
     @param[in] head_target, the current target.
  */
  void getRectInfo(std::vector<Target> &head_target);

  /* @brief Filter the direction of the rectangle to get better direction.
     @param[in] head_target, the current target.
  */
  void filterDirection(std::vector<Target> &head_target);

  /* @brief Calculate the sign of the x.
     @param[in] x, the int type number.
     @ret the sign of x.
  */
  int sgn(const int x);

  /* @brief Test function, not used now.
   */
  void saveTxt(std::vector<Target> &head_target);
  void generateTrackfile(void);

  string id_header_;   // The id for show in rviz
  int all_target_num_; // The total target number.
  float t_velocity_update_; // The unit update time of self vehicle.
  size_t size_current_targets_; // The size of current targets.
  double  t_absolute_get_data;//the time when the pcap is got unit s.
  std::ofstream trackfile_; // The file save the tracking information.
  std::string pkg_path_; // The tracking package path.
  Eigen::Matrix4f transformation_; // The transformation matrix from current frame to last frame.
  size_t tracking_time_; // The tracking times, also call the frames number
  float small_threshold_; // The small distance threshold in the first tracking.
  float big_threshold_; // The big distance threshold in the second tracking.
  int vel_mode_;     // The mode determine use the obd or mapping velocity file.
  std::string vel_file_path_; // The velocity file path.
  std::string obd_file_path_; // The obd velocity file path.
  std::string map_file_path_; // The mapping velocity file path.
  std::vector<double> self_velocity_map_; // The vector store the time, x, y, z, roll, pitch, yaw, vx, vy, vz, in the mapping file in one timestamp.
  std::vector<std::string> string_velocity_vector_; // The vector of velocity store the whole mapping velocity file information.
  const float PI_; // The pi, temporarily exists
  float T_LIDAR_UPDATE_; // The unit update time of lidar.
  const unsigned int TRACKING_VANISH_TIME_; // The vanish time to determine whether the target disappear or not.
};

}//end Robosense namespace

#endif
