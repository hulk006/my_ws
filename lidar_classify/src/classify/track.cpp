#include "track.h"
#define VERIFY 0
namespace Robosense
{

Target::Target()
{
}

Target::~Target()
{
}

Target::Target(const Target& target)
{
  initTarget(target);
}

Target& Target::operator=(const Target& target)
{
  if(this == &target)
  {
    return *this;
  }
  else
  {
    initTarget(target);
    return *this;
  }
}
void Target::initTarget(const Target& target)
{
  this->center_ = target.center_;
  this->classify_ = target.classify_;
  this->classify_probability_ = target.classify_probability_;
  this->cube_ = target.cube_;
  this->distance_ = target.distance_;
  this->flag_match_ = target.flag_match_;
  this->istracking_ = target.istracking_;
  this->length_ = target.length_;
  this->match_location_ = target.match_location_;
  this->match_id_ = target.match_id_;
  this->v_mean_absolute_ = target.v_mean_absolute_;
  this->pnumber_ = target.pnumber_;
  this->pointcloud_ = target.pointcloud_;
  this->rectangle_[0] = target.rectangle_[0];
  this->rectangle_[1] = target.rectangle_[1];
  this->rectangle_[2] = target.rectangle_[2];
  this->rectangle_[3] = target.rectangle_[3];
  this->rect_info_ = target.rect_info_;
  this->state_ = target.state_;
  this->vanish_time_ = target.vanish_time_;
  this->v_absolute_ = target.v_absolute_;
  this->v_angle_relative_ = target.v_angle_relative_;
  this->v_angle_absolute_ = target.v_angle_absolute_;
  this->v_angle_in_own_xaxis_ = target.v_angle_in_own_xaxis_;
  this->vx_relative_ = target.vx_relative_;
  this->vx_mean_relative_ = target.vx_mean_relative_;
  this->vy_relative_ = target.vy_relative_;
  this->vx_absolute_ = target.vx_absolute_;
  this->vy_absolute_ = target.vy_absolute_;
  this->width_ = target.width_;
  //for test
  this->vx_direct = target.vx_direct;
  this->vx_kalman=target.vx_kalman;
  this->vy_kalman = target.vy_kalman;
  this->vy_direct = target.vy_direct;
}


TargetKalman::TargetKalman():target_buffer_(50)
{
}

TargetKalman::~TargetKalman()
{
}

void TargetKalman::initKalman(const float x, const float y, const float t)
{
  float xv = 0;
  float yv = 0;
  const int stateNum = 4;
  const int measureNum = 4;
  cvkalman_    = cvCreateKalman(stateNum, measureNum, 0);
  measurement_ = cvCreateMat(measureNum, 1, CV_32FC1);
  /* create matrix data */
  const float A[] = {
    1, t,0, 0,
    0, 1, 0, 0,
    0, 0, 1, t,
    0, 0, 0,  1
  };
  const float H[] = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };
  const float P[] = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };
  const float Q[] = {
    0.02, 0, 0, 0,
    0, 0.02, 0, 0,
    0, 0, 0.02, 0,
    0, 0, 0, 0.02
  };
  const float R[] = {
    0.4 ,0, 0,  0,
    0, 1.5, 0, 0,
    0, 0, 0.4, 0,
    0, 0, 0, 1.5
  };

  //transition_matrix
  memcpy(cvkalman_->transition_matrix->data.fl, A, sizeof(A));
  //measurement_matrix
  memcpy(cvkalman_->measurement_matrix->data.fl, H, sizeof(H));
  //process_noise_cov
  memcpy(cvkalman_->process_noise_cov->data.fl, Q, sizeof(Q));
  //预测误差的协方差P
  memcpy(cvkalman_->error_cov_post->data.fl, P, sizeof(P));
  //R为观测协方差阵
  memcpy(cvkalman_->measurement_noise_cov->data.fl, R, sizeof(R));
  /* choose initial state */
  cvkalman_->state_post->data.fl[0] = x;
  cvkalman_->state_post->data.fl[1] = xv;
  cvkalman_->state_post->data.fl[2] = y;
  cvkalman_->state_post->data.fl[3] = yv;
}

void TargetKalman::getTargetpoint(const float *last_frame_, const float *curent_frame)
{
  // for(int i = 0; i < 4; ++i)
  // {
  //   cvkalman_->state_post->data.fl[i] = last_frame_[i]; // assignment state_post, not state_pre
  // }
  cvKalmanPredict(cvkalman_, 0);
  for(int i = 0; i < 4; ++i)
  {
    measurement_->data.fl[i] = curent_frame[i];
  }
  cvKalmanCorrect(cvkalman_, measurement_);
}

void Tracking::convertMsgForTracking(const lidar_msg::clusterArray &msg, pcl::PointCloud<pcl::PointXYZI> &current_pointcloud, std::vector<Robosense::SegMsg> &tracking_msg)
{
  std::vector<pcl::PointCloud<pcl::PointXYZI> > seg_vec;
  tracking_msg.clear();
  seg_vec.clear();
  current_pointcloud.clear();
  int seg_num = msg.clusters.size();
  t_absolute_get_data = msg.header.stamp.toSec();

  pcl::PointXYZI point_xyzi;
  Robosense::SegMsg tmp_msg;
  Robosense::SegResult seg_result;
  for(int i = 0; i < seg_num; ++i)
  {
    if(msg.clusters[i].x.size() < 20) // 20 ~ 5000 points cover the cluster size in statistics
    {
      continue;
    }
    if(!msg.clusters[i].inROI)
    {
      for(int j = 0; j < msg.clusters[i].x.size(); ++j)
      {
        point_xyzi.x = msg.clusters[i].x[j];
        point_xyzi.y = msg.clusters[i].y[j];
        point_xyzi.z = msg.clusters[i].z[j];
        point_xyzi.intensity = msg.clusters[i].i[j];
        current_pointcloud.points.push_back(point_xyzi);
      }
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZI> tmp_pcd;
      tmp_pcd.clear();
      for(int j = 0; j < msg.clusters[i].x.size(); ++j)
      {
        point_xyzi.x = msg.clusters[i].x[j];
        point_xyzi.y = msg.clusters[i].y[j];
        point_xyzi.z = msg.clusters[i].z[j];
        point_xyzi.intensity = msg.clusters[i].i[j];
        current_pointcloud.points.push_back(point_xyzi);
        float dis = sqrt(point_xyzi.x*point_xyzi.x + point_xyzi.y*point_xyzi.y + point_xyzi.z*point_xyzi.z);
        if(dis > 0.1)
        {
          tmp_pcd.push_back(point_xyzi);
        }
      }
      seg_vec.push_back(tmp_pcd);
      seg_result = getMinMaxSeg(tmp_pcd);
      tmp_msg.pointcloud  = tmp_pcd;
      tmp_msg.rect_label  = 0;
      tmp_msg.rect_length = seg_result.max_x - seg_result.min_x;
      tmp_msg.rect_width  = seg_result.max_y - seg_result.min_y;
      tmp_msg.seg_result.max_x = seg_result.max_x;
      tmp_msg.seg_result.min_x = seg_result.min_x;
      tmp_msg.seg_result.max_y = seg_result.max_y;
      tmp_msg.seg_result.min_y = seg_result.min_y;
      tmp_msg.seg_result.max_z = seg_result.max_z;
      tmp_msg.seg_result.min_z = seg_result.min_z;
      tracking_msg.push_back(tmp_msg);
    }
  }
}

Tracking::Tracking():PI_(3.1415926),                
                     T_LIDAR_UPDATE_(0.1),
                     yaw_(20)
{
  id_header_         = "map2";
  tracking_time_     = 0;
  small_threshold_   = 2.5;
  big_threshold_     = 3.3;
  all_target_num_    = 0;
  self_velocity_     = 0;
  t_velocity_update_ = 0.675;
  vel_mode_          = 0;
  pkg_path_ = ros::package::getPath("lidar_classify") + "/..";
  vel_file_path_     = pkg_path_ + "/data/obd.txt";
  map_file_path_ = pkg_path_ + "/data/0503vel.txt";
  obd_file_path_ = pkg_path_ + "/data/obd.txt";
  for(int i = 0; i < 10; ++i)
  {
    self_velocity_map_.push_back(0.0);
  }
  classify_flag_ = false;
#if VERIFY
   TRACKING_VANISH_TIME_ = 6;
#else
    TRACKING_VANISH_TIME_ = 10;
#endif
}

Tracking::~Tracking()
{
}

Robosense::SegResult Tracking::getMinMaxSeg(pcl::PointCloud<pcl::PointXYZI> vec)
{
  SegResult seg_result;
  float max_x = vec.at(0).x;
  float max_y = vec.at(0).y;
  float max_z = vec.at(0).z;
  float min_x = vec.at(0).x;
  float min_y = vec.at(0).y;
  float min_z = vec.at(0).z;
  for(int kk = 0; kk < vec.size(); ++kk)
  {
    if(vec.at(kk).x > max_x)
    {
      max_x = vec.at(kk).x;
    }
    if(vec.at(kk).x < min_x)
    {
      min_x = vec.at(kk).x;
    }
    if(vec.at(kk).y > max_y)
    {
      max_y = vec.at(kk).y;
    }
    if(vec.at(kk).y < min_y)
    {
      min_y = vec.at(kk).y;
    }
    if(vec.at(kk).z > max_z)
    {
      max_z = vec.at(kk).z;
    }
    if(vec.at(kk).z < min_z)
    {
      min_z =vec.at(kk).z;
    }
  }
  seg_result.max_x = max_x;
  seg_result.min_x = min_x;
  seg_result.max_y = max_y;
  seg_result.min_y = min_y;
  seg_result.max_z = max_z;
  seg_result.min_z = min_z;
  return seg_result;
}

void Tracking::initForTracking(const std::vector<Robosense::SegMsg> &seg_msg)
{
  target_current_.clear();
  Target rece_target;
  cv::Point2f center_rect;
  for(int i = 0; i < seg_msg.size(); ++i)
  {
      //when yan zheng velocity add the  && seg_msg[i].rect_width <= 5

#if VERIFY
  if((seg_msg[i].rect_length + seg_msg[i].rect_width) < 20 && seg_msg[i].rect_width <= 5 ) // when the seg width plus length less than 20 meters
#else
  if((seg_msg[i].rect_length + seg_msg[i].rect_width) < 20 ) // when the seg width plus length less than 20 meters
#endif
    {
      center_rect.x = (seg_msg[i].seg_result.max_x + seg_msg[i].seg_result.min_x) / 2;
      center_rect.y = (seg_msg[i].seg_result.max_y + seg_msg[i].seg_result.min_y) / 2;
      rece_target.center_.x   = center_rect.x;
      rece_target.center_.y   = center_rect.y;
      rece_target.width_      = seg_msg[i].rect_width;
      rece_target.length_     = seg_msg[i].rect_length;
      rece_target.classify_   = seg_msg[i].rect_label; // when init, rect_lable is 0
      rece_target.distance_   = sqrt(center_rect.x*center_rect.x + center_rect.y*center_rect.y);
      rece_target.pointcloud_ = seg_msg[i].pointcloud;
      rece_target.pnumber_    = rece_target.pointcloud_.size();
      rece_target.cube_       = seg_msg[i].seg_result;

      rece_target.rectangle_[0].x = rece_target.center_.x - 0.5 * rece_target.width_; // left down corner
      rece_target.rectangle_[0].y = rece_target.center_.y - 0.5 * rece_target.length_;
      rece_target.rectangle_[1].x = rece_target.center_.x + 0.5 * rece_target.width_; // right down corner
      rece_target.rectangle_[1].y = rece_target.center_.y - 0.5 * rece_target.length_;
      rece_target.rectangle_[2].x = rece_target.center_.x + 0.5 * rece_target.width_; // right up corner
      rece_target.rectangle_[2].y = rece_target.center_.y + 0.5 * rece_target.length_;
      rece_target.rectangle_[3].x = rece_target.center_.x - 0.5 * rece_target.width_; // left up corner
      rece_target.rectangle_[3].y = rece_target.center_.y + 0.5 * rece_target.length_;
      target_current_.push_back(rece_target);
    }
  }

  static int n = 1;
  size_current_targets_ = target_current_.size();
  for (int j = 0; j < target_current_.size(); ++j)
  {
    target_current_[j].flag_match_       = 0;
    target_current_[j].istracking_       = 0;
    target_current_[j].state_            = 0;
    target_current_[j].v_absolute_         = 0;
    target_current_[j].v_angle_relative_          = PI_/2;
    target_current_[j].v_angle_absolute_   = PI_/2;
    target_current_[j].v_angle_in_own_xaxis_ = PI_/2;
    target_current_[j].vanish_time_      = 0;
    target_current_[j].v_mean_absolute_           = 0;
    target_current_[j].vx_mean_relative_         = 0;
    //for test
    target_current_[j].vx_direct     = 0;
    target_current_[j].vy_direct     = 0;
    target_current_[j].vx_kalman     = 0;
    target_current_[j].vx_kalman     = 0;
    //for test
    target_current_[j].vx_relative_                   = 0;
    target_current_[j].vy_relative_                   = 0;
    target_current_[j].vx_absolute_         = 0;
    target_current_[j].vy_absolute_         = 0;
    target_current_[j].match_location_   =-1;
    target_current_[j].match_id_     = j;
  }

  if(tracking_time_ == n*1000) // free vector memory
  {
    std::vector<Robosense::TargetKalman> temp;
    temp = tracks_vector_;
    tracks_vector_.clear();
    tracks_vector_.swap(temp);
    ++n;
  }
  std::cout<<"yaw"<<yaw_.size() << std::endl;
  if(tracking_time_ == 0)
  {
    std::cout<<"yaw"<<yaw_.size() << std::endl;
    for(int i = 0; i < target_current_.size(); ++i)
    {
      Robosense::TargetKalman tracks;
      tracks.target_buffer_.push_back(target_current_[i]);
      tracks.initKalman(target_current_[i].center_.x, target_current_[i].center_.y, T_LIDAR_UPDATE_);
      tracks_vector_.push_back(tracks);
    }
    all_target_num_ = tracks_vector_.size();
  }

  for(int i = 0; i < tracks_vector_.size(); ++i)
  {
    tracks_vector_[i].target_buffer_[tracks_vector_[i].target_buffer_.size() - 1].flag_match_ = 0;
    tracks_vector_[i].target_buffer_[tracks_vector_[i].target_buffer_.size() - 1].match_location_ = -1;
  }
  ++tracking_time_;
}

void Tracking::deleteVanishTarget(void)
{
  std::vector<Robosense::TargetKalman>::iterator ip;
  for(int i = 0; i < tracks_vector_.size(); ++i)
  {
    size_t last = tracks_vector_[i].target_buffer_.size() - 1;
    if(tracks_vector_[i].target_buffer_[last].state_ == 2)
    {
      cvReleaseKalman(&tracks_vector_[i].cvkalman_);
      ip = tracks_vector_.begin() + i;
      tracks_vector_.erase(ip);
    }
  }
}

void Tracking::calculateTransformation(const float &self_v)
{
  transformation_(0,0) = 1; transformation_(0,1) = 0; transformation_(0,2) = 0; transformation_(0,3) = 0;
  transformation_(1,0) = 0; transformation_(1,1) = 1; transformation_(1,2) = 0; transformation_(1,3) = self_v * T_LIDAR_UPDATE_;
  transformation_(2,0) = 0; transformation_(2,1) = 0; transformation_(2,2) = 1; transformation_(2,3) = 0;
  transformation_(3,0) = 0; transformation_(3,1) = 0; transformation_(3,2) = 0; transformation_(3,3) = 1;
}

void Tracking::trackSegMain(const std::vector<Robosense::SegMsg> &seg_msg)
{
  initForTracking(seg_msg); // initialize current target
  readSelfVelocity(); // read the self car velocity from file
  calculateTransformation(self_velocity_);
  deleteVanishTarget(); // delete the vanish target and update the vector
  getRectInfo(target_current_);
  trackTarget(target_current_);
  trackVanishTarget();//
  smoothFilter(target_current_);
  int a=1;
  tracking_time_;
  trackLowVelTarget();
  filterDirection(target_current_);
  addCurrentTarget(target_current_); // add new target
  releaseTarget();
  // generateTrackfile();
#if VERIFY
   saveTxt(target_current_);
#endif

}

void Tracking::addCurrentTarget(std::vector<Target> &head_target)
{
  int new_target_num = 0;
  for(int i = 0; i < head_target.size(); ++i)
  {
    if(head_target[i].flag_match_ == 0)
    {
      Robosense::TargetKalman new_track;
      head_target[i].match_id_ = all_target_num_ + new_target_num;
      new_track.target_buffer_.push_back(head_target[i]);
      new_track.initKalman(head_target[i].center_.x, head_target[i].center_.y, T_LIDAR_UPDATE_);
      tracks_vector_.push_back(new_track);
      ++new_target_num;
    }
  }
  all_target_num_ = all_target_num_ + new_target_num;
}

int Tracking::sgn(const int x)
{
  if(x > 0)
  {
    return 1;
  }
  else if(x == 0)
  {
    return 0;
  }
  else if(x < 0)
  {
    return -1;
  }
}

void Tracking::releaseTarget(void)
{
}

void Tracking::getVelocityCallback(const std_msgs::Float32MultiArray &msg)
{
  self_velocity_ = msg.data[0] / 3.6;//m/s
}


float Tracking::caculateVelocity(Target & target)
{
  //vy只用vy相减**************************************
  //self_v m/s obd read
  // vy = vy + self_v;
  float yaw      = self_velocity_map_[6];
  float self_vx = 0;
  float self_vy = 0;
  if(vel_mode_ == true)
  {
    self_vx = self_velocity_map_[7];
    self_vy = self_velocity_map_[8];
  }
  else
  {
    self_vx = self_velocity_ *cos(yaw);
    self_vy = self_velocity_ *sin(yaw);
  }

  float vx = target.vx_relative_;//target velocty  in  own coordinate
  float vy = target.vy_relative_;

  float vx_turn = vy;
  float vy_turn = vx;

  float vx_rotated = 0;
  float vy_rotated = 0;
  vx_rotated = vx_turn*cos(yaw) - vy_turn * sin(yaw);
  vy_rotated = vx_turn*sin(yaw) + vy_turn * cos(yaw);
  target.vx_absolute_ = vx_rotated + self_vx;
  target.vy_absolute_ = vy_rotated + self_vy;


  target.v_angle_absolute_ = atan2(target.vy_absolute_ , target.vx_absolute_ );//map coordinate angle
  target.v_angle_in_own_xaxis_ = PI_/2 - (target.v_angle_absolute_ - yaw);

  if(target.v_angle_in_own_xaxis_ > PI_)
  {
    target.v_angle_in_own_xaxis_  = target.v_angle_in_own_xaxis_  -2*PI_;
  }


  float target_v = fabs(sqrt( target.vx_absolute_* target.vx_absolute_ + target.vy_absolute_* target.vy_absolute_));
  return target_v;
}

void Tracking::trackVanishTarget(void)
{
  std::vector<Robosense::TargetKalman>::iterator ip;
  Target temp_target;
  for(ip = tracks_vector_.begin(); ip != tracks_vector_.end(); ++ip)
  {
    int last = (*ip).target_buffer_.size() - 1;
    if((*ip).target_buffer_[last].state_ == 1 && (*ip).target_buffer_[last].vanish_time_ >= 1 && (*ip).target_buffer_[last].vanish_time_ <= TRACKING_VANISH_TIME_)
    {
      temp_target = (*ip).target_buffer_[last];
      float lastframe[] = {(float)(*ip).target_buffer_[last].center_.x, (*ip).target_buffer_[last].vx_relative_,(float)(*ip).target_buffer_[last].center_.y, (*ip).target_buffer_[last].vy_relative_};
      float x = (float)(*ip).target_buffer_[last].center_.x;
      float y = (float)(*ip).target_buffer_[last].center_.y;
      float currentframe[]={ x, (*ip).target_buffer_[last].vx_relative_, y, (*ip).target_buffer_[last].vy_relative_};

      (*ip).getTargetpoint(lastframe, currentframe);
      temp_target.center_.x = (*ip).cvkalman_->state_post->data.fl[0];
      temp_target.vx_relative_      = (*ip).cvkalman_->state_post->data.fl[1];
      temp_target.center_.y = (*ip).cvkalman_->state_post->data.fl[2];
      temp_target.vy_relative_      = (*ip).cvkalman_->state_post->data.fl[3];
      (*ip).target_buffer_.push_back(temp_target);
    }
  }
}

void Tracking::trackTarget(std::vector<Target> &head_target)
{
  Target temp_target;
  std::vector<Robosense::TargetKalman>::iterator ip;
  int location = 0;
  for(ip = tracks_vector_.begin(); ip != tracks_vector_.end(); ++ip)
  {
    int last = (*ip).target_buffer_.size() - 1;
    temp_target = (*ip).target_buffer_[last];
    for(int i = 0; i < head_target.size(); ++i)
    {
      if(head_target[i].flag_match_ != 1 && (*ip).target_buffer_[last].flag_match_ != 1 && head_target[i].distance_ > 0.6)
      {
        // move to last frame coordination
        cv::Point2f correct_center;
        correct_center.x = head_target[i].center_.x;
        correct_center.y = head_target[i].center_.y + T_LIDAR_UPDATE_ * self_velocity_;

        // the curent frame move to the last frame
        cv::Point2f v(0,0);
        v.x = correct_center.x - (*ip).target_buffer_[last].center_.x;
        v.y = correct_center.y - (*ip).target_buffer_[last].center_.y;

        float two_target_distance  = (v.x*v.x + v.y*v.y);//distance_ between the two frame
        float last_length  = (*ip).target_buffer_[last].length_;
        float last_width   = (*ip).target_buffer_[last].width_;
        float last_pnumber = (*ip).target_buffer_[last].pnumber_;
        float head_length  = head_target[i].length_;
        float head_width   = head_target[i].width_;
        float head_pnumber = head_target[i].pnumber_;

        float last_size = last_length + last_width;
        float head_size = head_length + head_width;
        float size_bias = fabs(head_size - last_size);
        float head_wdl  = head_width / head_length;// wdl width divide length
        float last_wdl  = last_width / last_length;// wdl width divide length
        float length_width_bias = head_wdl / last_wdl;
        int pnumber_bias = fabs(head_pnumber - last_pnumber);
        int number_threhold = head_pnumber > last_pnumber ? 0.4*head_pnumber: 0.4*last_pnumber;
        int size_threhold = 3;
        float dist_threhold = small_threshold_*small_threshold_;//2.5m

        bool dist_ok    = two_target_distance <= dist_threhold;
        bool size_ok    = size_bias < size_threhold;
        bool pnumber_ok = pnumber_bias < number_threhold;
        bool length_width_ok = length_width_bias > 0.5 && length_width_bias < 2;

        // if match the threhold, do the tracking

        if(dist_ok && size_ok && pnumber_ok && length_width_ok)
        {
          head_target[i].match_location_ = location;
          temp_target.match_location_ = i;
          doMatch(head_target[i], ip ,temp_target, v);
          continue;
        }

      }
    }
    location++;
  }
  //match again !! only use distance
  // TO think about ,second match  mainly aim at the head_target!!!!!
   int location_temp=0;
   for(ip = tracks_vector_.begin(); ip != tracks_vector_.end(); ++ip)
  {
     int last = (*ip).target_buffer_.size() - 1;
     temp_target = (*ip).target_buffer_[last];
    if(temp_target.flag_match_==0)
    {
      for(int i= 0;i <head_target.size(); ++i)
      {
        int last = (*ip).target_buffer_.size() - 1;
        temp_target = (*ip).target_buffer_[last];
        if(head_target[i].state_ ==0 && head_target[i].flag_match_ == 0)
        {
          //move to last frame coordination
          cv::Point2f correct_center;
          correct_center.x = head_target[i].center_.x;
          correct_center.y = head_target[i].center_.y + T_LIDAR_UPDATE_ * self_velocity_;
          // the curent frame move to the last frame
          cv::Point2f v(0,0);
          v.x = correct_center.x - (*ip).target_buffer_[last].center_.x;
          v.y = correct_center.y - (*ip).target_buffer_[last].center_.y;
          float two_target_distance = (v.x*v.x + v.y * v.y);
          float dist_threhold = big_threshold_*big_threshold_;//m
          bool dist_ok    = (two_target_distance <= dist_threhold);
          if(dist_ok)
          {
            temp_target.match_location_ = i;
            head_target[i].match_location_ = location_temp;
            doMatch(head_target[i], ip ,temp_target, v);
          }
        }

      }
    }
    location_temp ++;
  }
// judge the tracks' state
  stateJudgement();
}

void Tracking::doMatch( Target &current_target , std::vector<Robosense::TargetKalman>::iterator &ip,Target &temp_target, cv::Point2f v )
{
  // if match the threhold, do the tracking
  int last = (*ip).target_buffer_.size() - 1;
  Robosense::Target last_target =  (*ip).target_buffer_[last];
  float delta_width  = current_target.width_ - last_target.width_;
  float delta_length = current_target.length_ - last_target.length_;
  // ajust the center of the rectangle
  cv::Point2f center(0, 0);
  center.x = current_target.center_.x - sgn(v.x)*delta_width / 2;
  center.y = current_target.center_.y - sgn(v.y)*delta_length / 2;

  // the center diff after ajustment
  v.x = center.x - last_target.center_.x;
  v.y = center.y - last_target.center_.y;
  // the velocity after ajustment
  current_target.vx_relative_ = v.x / T_LIDAR_UPDATE_;
  current_target.vy_relative_ = v.y / T_LIDAR_UPDATE_;
  current_target.center_ = center;

  float two_target_distance = sqrt(v.x*v.x + v.y*v.y);
  //if(two_target_distance <= 0.2) // if the center dist between last and current frame is less 0.2, consider static
  {
    current_target.v_angle_relative_ = 0;
    current_target.v_angle_absolute_ = 0;
    current_target.v_absolute_ = 0;
    current_target.vx_relative_ = 0;
    current_target.vy_relative_ = 0;
      //for test
    current_target.vx_direct      = 0;
    current_target.vy_direct      = 0;
    current_target.vx_kalman   = 0;
    current_target.vy_kalman   = 0;
  }
  //else
  {
    current_target.v_angle_relative_ = atan2( v.y, v.x );
    current_target.v_absolute_ = two_target_distance / T_LIDAR_UPDATE_;
    current_target.vx_relative_ = v.x / T_LIDAR_UPDATE_;
    current_target.vy_relative_ = v.y / T_LIDAR_UPDATE_;
    //for test
    current_target.vx_direct      = v.x / T_LIDAR_UPDATE_;
    current_target.vy_direct      = v.y / T_LIDAR_UPDATE_;
  }


  temp_target.classify_ = current_target.classify_;
  temp_target.cube_     = current_target.cube_;
  temp_target.distance_ = current_target.distance_;
  temp_target.state_    = 1; // tracked
  temp_target.match_id_ =  last_target.match_id_;
  temp_target.pnumber_    = current_target.pnumber_;
  temp_target.pointcloud_ = current_target.pointcloud_;
  temp_target.width_      = current_target.width_;
  temp_target.length_     = current_target.length_;
  temp_target.vanish_time_ = 0; // tracked is no disappear
  temp_target.rectangle_[0] = current_target.rectangle_[0];
  temp_target.rectangle_[1] = current_target.rectangle_[1];
  temp_target.rectangle_[2] = current_target.rectangle_[2];
  temp_target.rectangle_[3] = current_target.rectangle_[3];
  temp_target.flag_match_ = 1;
  temp_target.rect_info_ = current_target.rect_info_;

  current_target.state_ = 1;
  current_target.flag_match_ = 1;

  current_target.match_id_ =  last_target.match_id_;

  float lastframe[] = {(float) last_target.center_.x, last_target.vx_relative_, (float) last_target.center_.y, last_target.vy_relative_};
  float currentframe[] = {(float) current_target.center_.x, current_target.vx_relative_, (float) current_target.center_.y, current_target.vy_relative_};

  (*ip).getTargetpoint(lastframe, currentframe);
  // after the kalman filter
  temp_target.center_.x = (*ip).cvkalman_->state_post->data.fl[0];
  temp_target.center_.y = (*ip).cvkalman_->state_post->data.fl[2];
  temp_target.vx_relative_      = (*ip).cvkalman_->state_post->data.fl[1];
  temp_target.vy_relative_      = (*ip).cvkalman_->state_post->data.fl[3];
  //for test
  current_target.vx_kalman = temp_target.vx_relative_ ;
  current_target.vy_kalman = temp_target.vy_relative_ ;


  float velocity_angle  = atan2(temp_target.vy_relative_, temp_target.vx_relative_);
  // absolute velocity
  float abs_v = caculateVelocity(temp_target);//m/s
  //temp_target.v_angle_absolute_ = atan2(temp_target.v_y_, temp_target.v_x_);
  temp_target.v_absolute_ = abs_v;//m/s
  temp_target.v_angle_relative_  = velocity_angle;
  current_target.v_angle_relative_ = velocity_angle;
  current_target.v_angle_absolute_ = temp_target.v_angle_absolute_;
  current_target.vx_relative_ = temp_target.vx_relative_;
  current_target.vy_relative_ = temp_target.vy_relative_;
  current_target.vx_absolute_ = temp_target.vx_absolute_;
  current_target.vy_absolute_ = temp_target.vx_absolute_;
  current_target.v_angle_in_own_xaxis_ = temp_target.v_angle_in_own_xaxis_;
  current_target.v_absolute_ = abs_v;
  current_target.v_mean_absolute_   = abs_v;//m/s
  temp_target.istracking_ = 1;
  //for test
  temp_target.vx_direct = current_target.vx_direct;
  temp_target.vy_direct = current_target.vy_direct;
  temp_target.vx_kalman = current_target.vx_kalman;
  temp_target.vy_kalman = current_target.vy_kalman;
  // current_target.match_location_ = location;
  //temp_target.match_location_ = i;
  (*ip).target_buffer_.push_back(temp_target);
}

void Tracking::stateJudgement(void)
{
  for(int i = 0; i < tracks_vector_.size(); ++i)
  {
    size_t last = tracks_vector_[i].target_buffer_.size() - 1;
    if(tracks_vector_[i].target_buffer_[last].flag_match_ == 0)
    {
      tracks_vector_[i].target_buffer_[last].vanish_time_++; // vanish time increase
    }
    else
    {
      //keep  tracks_vector_[i].target_buffer_[last].vanish_time_ = 1;
    }

    if( tracks_vector_[i].target_buffer_[last].vanish_time_ >= TRACKING_VANISH_TIME_)
    {
      tracks_vector_[i].target_buffer_[last].state_ = 2;
    }
    else
    {
      tracks_vector_[i].target_buffer_[last].state_ = 1; // vanish for a while, keep tracking
    }
  }

}

void Tracking::trackLowVelTarget(void)
{
  for(int i = 0; i < tracks_vector_.size(); ++i)
  {
    float mean_temp=0;
    int target_num_mean = tracks_vector_[i].target_buffer_.size() > 10 ? 10: tracks_vector_[i].target_buffer_.size();
    size_t last = tracks_vector_[i].target_buffer_.size() - 1;
    if(tracks_vector_[i].target_buffer_.size() >= target_num_mean)
    {
      for(int j = tracks_vector_[i].target_buffer_.size() - target_num_mean; j < tracks_vector_[i].target_buffer_.size(); ++j)
      {
        mean_temp = mean_temp + tracks_vector_[i].target_buffer_[j].v_absolute_;
      }
      tracks_vector_[i].target_buffer_[last].v_mean_absolute_ = mean_temp/ target_num_mean;//km/h
      if(tracks_vector_[i].target_buffer_[last].match_location_ >= 0)
      {
        if(tracks_vector_[i].target_buffer_[last].v_mean_absolute_ <= 5) // 5 //km/h
        {
          target_current_[ tracks_vector_[i].target_buffer_[last].match_location_ ].v_mean_absolute_ = tracks_vector_[i].target_buffer_[last].v_mean_absolute_;
        }
      }
      else
      {
      }
    }
  }
}

void Tracking::getRectInfo(std::vector<Target> &head_target)
{
  for(int i = 0; i < head_target.size(); ++i)
  {
    if(head_target[i].length_ > 1 || head_target[i].width_ > 1)
    {
      SegResult seg_result = head_target[i].cube_;
      float xmin = seg_result.min_x;
      float ymin = seg_result.min_y;
      std::vector<cv::Point2f> target_points_vec;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      *cloud = head_target[i].pointcloud_;
      float deta_y = head_target[i].length_;
      float deta_x = head_target[i].width_;
      for (int j = 0; j < cloud->points.size(); ++j)
      {
        cv::Point2f target_point;
        target_point.x = cloud->points[j].x - xmin;
        target_point.y = cloud->points[j].y - ymin;
        target_points_vec.push_back(target_point);
      }

      std::vector<std::vector<cv::Point2f> > target_points_doublevec;
      target_points_doublevec.push_back(target_points_vec);
      BoxEstimater rect;
      std::vector<Robosense::rect_info> target_rectangle;
      rect.Rotated_point(target_points_doublevec, target_rectangle, (int)20 * deta_y + 1, (int)20 * deta_x + 1);
      for(int j = 0; j < 4; ++j)
      {
        target_rectangle[0].four_points[j].x += xmin;
        target_rectangle[0].four_points[j].y += ymin;
      }
      target_rectangle[0].center.x += xmin;
      target_rectangle[0].center.y += ymin;
      target_rectangle[0].y_theta_mean = target_rectangle[0].y_theta;
      head_target[i].rect_info_ = target_rectangle[0];
    }
    else
    {
      head_target[i].rect_info_.four_points.clear();
      head_target[i].rect_info_.center = head_target[i].center_;
      head_target[i].rect_info_.width  = head_target[i].width_;
      head_target[i].rect_info_.length = head_target[i].length_;
      head_target[i].rect_info_.y_theta = 0;
      head_target[i].rect_info_.main_derection = PI_/2;
      head_target[i].rect_info_.y_theta_mean = 0;
    }
  }
}

void Tracking::filterDirection(std::vector<Target> &head_target)
{
  int space = 2;
  for( int i = 0; i < tracks_vector_.size(); ++i)
  {
    if(tracks_vector_[i].target_buffer_.size() >= space)
    {
      for( int j = 1; j < tracks_vector_[i].target_buffer_.size(); ++j)
      {
        float deta = tracks_vector_[i].target_buffer_[j].rect_info_.y_theta - tracks_vector_[i].target_buffer_[j-1].rect_info_.y_theta_mean;
        if(fabs(deta) > PI_*0.5)
        {
          tracks_vector_[i].target_buffer_[j].rect_info_.y_theta = fabs(fabs(tracks_vector_[i].target_buffer_[j].rect_info_.y_theta) - PI_);
          float delta_2 = tracks_vector_[i].target_buffer_[j].rect_info_.y_theta - tracks_vector_[i].target_buffer_[j-1].rect_info_.y_theta_mean;
          if (fabs(delta_2>PI_*0.25))
          {
            tracks_vector_[i].target_buffer_[j].rect_info_.y_theta = tracks_vector_[i].target_buffer_[j-1].rect_info_.y_theta_mean+delta_2*0.5;
          }
        }
      }
      int last = tracks_vector_[i].target_buffer_.size() - 1;
      int corres_lowmark = tracks_vector_[i].target_buffer_[last].match_location_;// find the correspone target in the target buffer
      for( int j = 1; j< tracks_vector_[i].target_buffer_.size(); ++j)
      {
        tracks_vector_[i].target_buffer_[j].rect_info_.y_theta_mean = 0.5*tracks_vector_[i].target_buffer_[j-1].rect_info_.y_theta_mean
          + 0.5*tracks_vector_[i].target_buffer_[j].rect_info_.y_theta;
        tracks_vector_[i].target_buffer_[j].vx_mean_relative_ =  0.5*tracks_vector_[i].target_buffer_[j-1].vx_mean_relative_ + 0.5*tracks_vector_[i].target_buffer_[j].vx_relative_;
      }
      if(tracks_vector_[i].target_buffer_[last].match_location_ >= 0)
      {
        head_target[corres_lowmark].vx_mean_relative_ = tracks_vector_[i].target_buffer_[last].vx_mean_relative_;
        head_target[corres_lowmark].rect_info_.y_theta_mean = tracks_vector_[i].target_buffer_[last].rect_info_.y_theta_mean;
      }
    }
    else
    {
      //vx_mean_relative_ and y_theta_mean keep  unchangeable
    }
  }
}

void Tracking::smoothFilter(std::vector<Target> &head_target)
{
  for(int i = 0; i < tracks_vector_.size(); ++i)
  {
    if(tracks_vector_[i].target_buffer_.size() >= 2)
    {
      int last = tracks_vector_[i].target_buffer_.size() -1;
      //vx_relative_ filter
      tracks_vector_[i].target_buffer_[last].vx_relative_ = 0.5*tracks_vector_[i].target_buffer_[last].vx_relative_ +
        0.5*tracks_vector_[i].target_buffer_[last-1].vx_relative_;
      tracks_vector_[i].target_buffer_[last].vy_relative_ = 0.5*tracks_vector_[i].target_buffer_[last].vy_relative_ +
        0.5*tracks_vector_[i].target_buffer_[last-1].vy_relative_;
      //v_angle_
      tracks_vector_[i].target_buffer_[last].v_angle_relative_ = 0.5*tracks_vector_[i].target_buffer_[last].v_angle_relative_ +
        0.5*tracks_vector_[i].target_buffer_[last-1].v_angle_relative_;

      tracks_vector_[i].target_buffer_[last].vx_absolute_ = 0.5*tracks_vector_[i].target_buffer_[last].vx_absolute_ +
        0.5*tracks_vector_[i].target_buffer_[last-1].vx_absolute_;
      tracks_vector_[i].target_buffer_[last].vy_absolute_ = 0.5*tracks_vector_[i].target_buffer_[last].vy_absolute_ +
        0.5*tracks_vector_[i].target_buffer_[last-1].vy_absolute_;
      tracks_vector_[i].target_buffer_[last].v_angle_absolute_ = 0.5*tracks_vector_[i].target_buffer_[last].v_angle_absolute_ +
        0.5*tracks_vector_[i].target_buffer_[last-1].v_angle_absolute_;
      tracks_vector_[i].target_buffer_[last].v_absolute_ = 0.5*tracks_vector_[i].target_buffer_[last].v_absolute_  +
        0.5*tracks_vector_[i].target_buffer_[last-1].v_absolute_ ;
      //tracks_vector_[i].target_buffer_[last].v_angle_in_own_xaxis_ = 0.5*tracks_vector_[i].target_buffer_[last].v_angle_in_own_xaxis_  +
       // 0.5*tracks_vector_[i].target_buffer_[last-1].v_angle_in_own_xaxis_ ;

      int corres_lowmark = tracks_vector_[i].target_buffer_[last].match_location_;// find the correspone target in the target buffer
      if(tracks_vector_[i].target_buffer_[last].match_location_ >= 0)
      {
        head_target[corres_lowmark].vx_relative_ = tracks_vector_[i].target_buffer_[last].vx_relative_;
        head_target[corres_lowmark].vy_relative_ = tracks_vector_[i].target_buffer_[last].vy_relative_;
        head_target[corres_lowmark].v_angle_relative_ = tracks_vector_[i].target_buffer_[last].v_angle_relative_;

        head_target[corres_lowmark].vx_absolute_ = tracks_vector_[i].target_buffer_[last].vx_absolute_;
        head_target[corres_lowmark].vy_absolute_ = tracks_vector_[i].target_buffer_[last].vy_absolute_;
        head_target[corres_lowmark].v_angle_absolute_ = tracks_vector_[i].target_buffer_[last].v_angle_absolute_;
        head_target[corres_lowmark].v_absolute_ = tracks_vector_[i].target_buffer_[last].v_absolute_;
        head_target[corres_lowmark].v_angle_in_own_xaxis_ = tracks_vector_[i].target_buffer_[last].v_angle_in_own_xaxis_;
      }

    }
    else
    {
      //do nothing for tracks_vector_
    }
  }

}

void Tracking::readSelfVelocity()
{
  if(vel_mode_ == 0)
  {
    readSelfVelocityObd();
    readSelfVelocityMap();
    std::cout<<" yaw"<<self_velocity_map_[6]<<std::endl;
  }
  else if(vel_mode_ == 1)
  {
    readSelfVelocityMap();
  }
  else
  {
    std::cout << "velocity file input error!" << std::endl;
  }
}

void Tracking::readSelfVelocityObd(void)
{

  int m = (tracking_time_ - 1) * 0.1 / t_velocity_update_ + 0.5;
  std::string string_path = obd_file_path_;
  std::ifstream fopen;
  fopen.open(string_path.c_str());
  if(!fopen)
  {
    std::cout << "obd_path is not exsit!" << std::endl;
  }
  else
  {

    std::string line;
    while(getline(fopen, line))
    {
      std::istringstream obd_info(line);
      double obd_time;
      float velocity;
      obd_info >> obd_time;
      obd_info >> velocity;
      obd_time = obd_time/1000000;
      if(fabs(obd_time - (t_absolute_get_data + 0.1)) <= t_velocity_update_/2 )
      {
          std::cout <<"tracking time=" <<tracking_time_<< " hang shu  "<< m<<std::endl;
          self_velocity_ = velocity/3.6;//m/s
      }
    }
  }
  fopen.close();
}

void Tracking::readSelfVelocityMap(void)
{
  static int count = 0;
  if (count == 0)
  {
    string_velocity_vector_.clear();
    std::string string_path = map_file_path_;
    std::ifstream map_velocity(string_path.c_str());
    std::string string_line;
    if (!map_velocity.is_open())
    {
      std::cout << "velocity file is not exsit!" << std::endl;
    }
    else
    {
      while(getline(map_velocity, string_line))
      {
        string_velocity_vector_.push_back(string_line);
      }
    }
    map_velocity.close();
  }

  if (string_velocity_vector_.size() > 0)
  {
    for(int i = count; i < string_velocity_vector_.size(); ++i)
    {
      std::istringstream record(string_velocity_vector_[i]);
      double velocity_info[10]; //velocity_info[10] represent the float x, y, z, roll, pitch, yaw, vx, vy, vz   + the first variable time ;
      //t, x, y, z, r, p, yaw, vx, vy, vz
      for(int j = 0; j < 10; ++j)
      {
        record >> velocity_info[j];
      }
      double time = velocity_info[0];
      if(abs(t_absolute_get_data - time) < 0.1)//match the time
      {
        count = i;
        self_velocity_map_.clear();
        for(int j = 0; j < 10; ++j)
        {
          self_velocity_map_.push_back(velocity_info[j]);
        }
        //self_velocity_ = sqrt(self_velocity_map_[7] * self_velocity_map_[7] +  self_velocity_map_[8]* self_velocity_map_[8]);
        float yaw = self_velocity_map_[6];
        yaw_.push_back(yaw);
        break;
      }
      else if((time - t_absolute_get_data) > 1.0)
      {
        break;
      }
    }
  }
}

void Tracking::saveTxt(std::vector<Target> &head_target)
{
  ros::Time time;
  time = ros::Time::now();
  std::string string_path = std::string(pkg_path_ + "/data/saveTxt/");
  for (int i = 0; i < tracks_vector_.size(); ++i)
  {
    std::stringstream ss;
    size_t last = tracks_vector_[i].target_buffer_.size() - 1;
    ss << string_path << tracks_vector_[i].target_buffer_[last].match_id_ << ".txt";

    ofstream record;
    record.open(ss.str().c_str(), std::ios::app);
    if(!record)
    {
      std::cout << "Unable to open otfile";
      ofstream record( ss.str().c_str(), std::ios::out);
    }
    record  << " " << tracking_time_
            << " " << tracks_vector_[i].target_buffer_[last].center_.x
           << " " << tracks_vector_[i].target_buffer_[last].center_.y
           << " " << tracks_vector_[i].target_buffer_[last].vx_relative_
           << " "<< tracks_vector_[i].target_buffer_[last].vy_relative_
           << " " << tracks_vector_[i].target_buffer_[last].v_absolute_
           <<" "<<tracks_vector_[i].target_buffer_[last].v_mean_absolute_

           << " " << tracks_vector_[i].target_buffer_[last].vx_direct
           << " " << tracks_vector_[i].target_buffer_[last].vy_direct
           << " " << tracks_vector_[i].target_buffer_[last].vx_kalman
            << " " << tracks_vector_[i].target_buffer_[last].vy_kalman
           << std::endl;
    record.close();
  }
}

void Tracking::generateTrackfile()
{
  std::vector<Robosense::TargetKalman>::iterator ip;
  for (int i = 0; i < tracks_vector_.size(); ++i)
  {
    size_t last = tracks_vector_[i].target_buffer_.size() - 1;
    if( tracks_vector_[i].target_buffer_[last].state_ == 2 && tracks_vector_[i].target_buffer_.size() > 15)
    {
      std::string string_path = std::string(pkg_path_ + "/data/trackfile.txt");
      const char *trackfile_name = string_path.c_str();
      trackfile_.open(trackfile_name, std::ios::app);

      trackfile_ << "Track"      << std::endl;
      trackfile_ << "label"      << std::endl;
      trackfile_ << "unlabled"   << std::endl;
      trackfile_ << "num_frames" << std::endl;
      boost::circular_buffer<Target> a_track;
      a_track = tracks_vector_[i].target_buffer_;
      trackfile_ << a_track.size() - 2 << std::endl;
      for(int j = 0; j < a_track.size() - 2; ++j)
      {
        Target target_in_atrack = a_track[j];
        trackfile_ << "seg"          << std::endl;
        trackfile_ << "segPointsNum" << std::endl;
        trackfile_ << target_in_atrack.pointcloud_.size() << std::endl;
        for(int ii = 0; ii < target_in_atrack.pointcloud_.size(); ++ii)
        {
          pcl::PointXYZI point = target_in_atrack.pointcloud_[ii];
          trackfile_.write((char*)&point.x, sizeof(double));
          trackfile_.write((char*)&point.y, sizeof(double));
          trackfile_.write((char*)&point.z, sizeof(double));
          trackfile_.write((char*)&point.intensity, sizeof(double));
          trackfile_ << std::endl;
        }
      }
      trackfile_.close();
    }
  }
}

void Tracking::setVelFilePath(const std::string& vel_file, int mode)
{
  if(mode == 0)
  {
    obd_file_path_ = vel_file;
    vel_mode_ = 0;
  }
  else if(mode == 1)
  {
    map_file_path_ = vel_file;
    vel_mode_ = 1;
  }
  else
  {
    std::cout << "velocity input error!" << std::endl;
  }
}
void Tracking::setVelFilePath(const std::string& obd_file,const std::string& mapping_file, int mode)
{
  if(mode ==0)
  {
    obd_file_path_ = obd_file;
    vel_mode_ = 0;
    map_file_path_ = mapping_file;
  }
  else
  {
    std::cout << " obd or map file open error!" << std::endl;
  }
}
}
