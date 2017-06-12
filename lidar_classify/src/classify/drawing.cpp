#include "drawing.h"
namespace Robosense
{
Drawing::Drawing():PI_(3.1415926)
{
  v_threhold_ = 5/3.6;
}
Drawing::~Drawing()
{
}
void Drawing::drawInRviz(ros::Publisher& vel_pub,ros::Publisher& pc_pub,const Robosense::Tracking& tracking, pcl::PointCloud<pcl::PointXYZI> &current_pointcloud)
{
  selfTurning(tracking);
  std::string id_header = "map2";
  std::size_t size_current_targets = tracking.target_current_.size();
  visualization_msgs::MarkerArray markerarray;
  for(int i = 0; i < size_current_targets; ++i)
  {
    if(tracking.target_current_[i].classify_ != Init)
    {
      visualization_msgs::Marker cuboid_marker;
      cuboid_marker.header.frame_id = id_header;
      cuboid_marker.header.stamp = ros::Time::now();
      cuboid_marker.ns = "target_rect";
      cuboid_marker.type = visualization_msgs::Marker::LINE_LIST;
      cuboid_marker.action = visualization_msgs::Marker::ADD;
      cuboid_marker.pose.orientation.x = 0.0;
      cuboid_marker.pose.orientation.y = 0.0;
      cuboid_marker.pose.orientation.z = 0.0;
      cuboid_marker.pose.orientation.w = 0.0;
      cuboid_marker.id = i;
      cuboid_marker.pose.position.x = 0;
      cuboid_marker.pose.position.y = 0;
      cuboid_marker.pose.position.z = 0;
      cuboid_marker.scale.x = 0.05;
      cuboid_marker.scale.y = 0.05;
      cuboid_marker.scale.z = 0.05;
      cuboid_marker.lifetime = ros::Duration(0.1);
      drawRect(cuboid_marker,tracking.target_current_[i]);

      visualization_msgs::Marker marker;
      marker.header.frame_id = id_header;
      marker.header.stamp = ros::Time::now();
      marker.ns = "target_velocity";
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.8;
      marker.scale.y = 0.8;
      marker.scale.z = 0.8;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(0.1);
      marker.id = i;
      int velocity = tracking.target_current_[i].v_mean_absolute_ * 3.6;
      int seta = tracking.target_current_[i].v_angle_relative_;
      int match_number = tracking.target_current_[i].match_id_%1000;
      cv::Point2i center;
      center = tracking.target_current_[i].center_;
      std::ostringstream  tempstring;

      switch (tracking.target_current_[i].classify_)
      {
        case Car:
        {
          if(velocity <= 5)
          {
            tempstring << match_number << " Car" << ""<< std::endl << "Loc.:" << "(" << center.x << "," << center.y<< ")";
          }
          else
          {
            tempstring << match_number << " Car" << " " << velocity << "km/h" << std::endl << "Loc.:" << "(" <<center.x << "," << center.y << ")";
          }
          break;
        }
        case Truck:
        {
          tempstring << match_number << " Truck" << " " << velocity << "km/h" << std::endl << "Loc.:" << "(" << center.x << "," << center.y << ")";
          break;
        }
        case Ped:
        {
          tempstring << match_number << " Ped" << " " << velocity << "km/h" << std::endl<< "Loc.:" << "(" << center.x << "," << center.y << ")";
          break;
        }
        case Unknow:
        {
          marker.color.r = 0.4f;
          marker.color.g = 0.0f;
          marker.color.b = 0.5f;
          tempstring << match_number << " unknow";
          break;
        }
        default:
        {
          std::cout << "Classify Error!" << std::endl;
          break;
        }
      }
      std::string v_text(tempstring.str());
      marker.text = v_text;
      marker.pose.position.x = (tracking.target_current_[i].cube_.max_x + tracking.target_current_[i].cube_.min_x)/2;
      marker.pose.position.y =  tracking.target_current_[i].cube_.max_y;
      marker.pose.position.z =  tracking.target_current_[i].cube_.max_z + 0.1;
      markerarray.markers.push_back(marker);
      markerarray.markers.push_back(cuboid_marker);
    }
  }
  for(int i = size_current_targets; i < 100; ++i)
   {
       visualization_msgs::Marker cuboid_marker;
       cuboid_marker.header.frame_id = id_header;
       cuboid_marker.header.stamp = ros::Time::now();
       cuboid_marker.ns = "target_rect";
       cuboid_marker.type = visualization_msgs::Marker::LINE_LIST;
//      cuboid_marker.action = visualization_msgs::Marker::ADD;
       cuboid_marker.pose.orientation.x = 0.0;
       cuboid_marker.pose.orientation.y = 0.0;
       cuboid_marker.pose.orientation.z = 0.0;
       cuboid_marker.pose.orientation.w = 0.0;
       cuboid_marker.id = i;
       cuboid_marker.pose.position.x=0;
       cuboid_marker.pose.position.y=0;
       cuboid_marker.pose.position.z=0;
       cuboid_marker.scale.x = 0.05;
       cuboid_marker.scale.y = 0.05;
       cuboid_marker.scale.z = 0.05;
//      cuboid_marker.lifetime = ros::Duration(0.1);
       cuboid_marker.color.r = 0.8f;
       cuboid_marker.color.g = 0.2f;
       cuboid_marker.color.b = 0.2f;
       cuboid_marker.color.a = 0.0;

       visualization_msgs::Marker marker;
       marker.header.frame_id = id_header;
       marker.header.stamp = ros::Time::now();
       marker.ns = "target_velocity";
       marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//      marker.action = visualization_msgs::Marker::ADD;
       marker.pose.orientation.x = 0.0;
       marker.pose.orientation.y = 0.0;
       marker.pose.orientation.z = 0.0;
       marker.pose.orientation.w = 1.0;
       marker.scale.x = 0.8;
       marker.scale.y = 0.8;
       marker.scale.z = 0.8;
       marker.color.r = 0.0f;
       marker.color.g = 1.0f;
       marker.color.b = 0.0f;
       marker.color.a = 0.0;
//      marker.lifetime = ros::Duration(0.1);
       marker.id = i;

       std::ostringstream  tempstring;
       tempstring << " unknow";
       std::string v_text(tempstring.str());
       marker.text = v_text;
       markerarray.markers.push_back(marker);
       markerarray.markers.push_back(cuboid_marker);
   }
  //自身车速;
  visualization_msgs::Marker selfmarker;
  selfmarker.header.frame_id = id_header;
  selfmarker.header.stamp = ros::Time::now();
  selfmarker.ns = "self_velocity";
  selfmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  selfmarker.action = visualization_msgs::Marker::ADD;
  selfmarker.pose.orientation.x = 0.0;
  selfmarker.pose.orientation.y = 0.0;
  selfmarker.pose.orientation.z = 0.0;
  selfmarker.pose.orientation.w = 1.0;
  selfmarker.scale.x = 1.1;
  selfmarker.scale.y = 1.1;
  selfmarker.scale.z = 1.1;
  selfmarker.color.r = 0.0f;
  selfmarker.color.g = 1.0f;
  selfmarker.color.b = 0.0f;
  selfmarker.color.a = 1.0;
  selfmarker.id = 0;
  int v = tracking.self_velocity_*3.6;
  std::ostringstream tempstring;
  tempstring << v << "km/h";
  std::string v_text(tempstring.str());
  selfmarker.text = v_text;
  selfmarker.pose.position.x = 0;
  selfmarker.pose.position.y = 0;
  selfmarker.pose.position.z = 0;
  markerarray.markers.push_back(selfmarker);
  vel_pub.publish(markerarray) ;
  current_pointcloud.header.frame_id = id_header;
  current_pointcloud.height = 1;
  current_pointcloud.width = current_pointcloud.points.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(current_pointcloud,output);
  pc_pub.publish(output);
}

void Drawing::drawRect(visualization_msgs::Marker &cuboid_marker, const Robosense::Target target)
{
  int label_predict = target.classify_;
  // color
  switch (label_predict)
  {
    case Car:
    {
      cuboid_marker.color.r = 0.8f;
      cuboid_marker.color.g = 0.2f;
      cuboid_marker.color.b = 0.2f;
      cuboid_marker.color.a = 1.0;
      break;
    }
    case Truck:
    {
      cuboid_marker.color.r = 0.2f;
      cuboid_marker.color.g = 0.2f;
      cuboid_marker.color.b = 0.8f;
      cuboid_marker.color.a = 1.0;
      break;
    }
    case Ped:
    {
      cuboid_marker.color.r = 0.0f;
      cuboid_marker.color.g = 1.0f;
      cuboid_marker.color.b = 1.0f;
      cuboid_marker.color.a = 1.0;
      break;
    }
    case Unknow:
    {
      cuboid_marker.color.r = 0.5f;
      cuboid_marker.color.g = 0.5f;
      cuboid_marker.color.b = 0.0f;
      cuboid_marker.color.a = 0.5;
      break;
    }
    default:
    {
      std::cout << "Classify Error!" << std::endl;
      break;
    }
  }
  // draw the cuboid
  SegResult seg_result = target.cube_;
  float x_theta = target.v_angle_in_own_xaxis_;
  float xmin = seg_result.min_x;
  float xmax = seg_result.max_x;
  float ymin = seg_result.min_y;
  float ymax = seg_result.max_y;
  geometry_msgs::Point cuboid[10];
  // default cuboid
  cuboid[4].x = xmin;  cuboid[4].y = ymin;  cuboid[4].z = seg_result.min_z;
  cuboid[5].x = xmax;  cuboid[5].y = ymin;  cuboid[5].z = seg_result.min_z;
  cuboid[6].x = xmax;  cuboid[6].y = ymax;  cuboid[6].z = seg_result.min_z;
  cuboid[7].x = xmin;  cuboid[7].y = ymax;  cuboid[7].z = seg_result.min_z;
  cuboid[0].x = xmin;  cuboid[0].y = ymin;  cuboid[0].z = seg_result.max_z;
  cuboid[1].x = xmax;  cuboid[1].y = ymin;  cuboid[1].z = seg_result.max_z;
  cuboid[2].x = xmax;  cuboid[2].y = ymax;  cuboid[2].z = seg_result.max_z;
  cuboid[3].x = xmin;  cuboid[3].y = ymax;  cuboid[3].z = seg_result.max_z;

  cv::Point2f center;
  center.x = (seg_result.max_x + seg_result.min_x) / 2;
  center.y = (seg_result.max_y + seg_result.min_y) / 2;
  switch (label_predict)
  {
    case Car:
    case Truck:
    {
      std::vector<Robosense::rect_info> target_rectangle;
      target_rectangle.clear();
      target_rectangle.push_back(target.rect_info_);
      predictVehicleRect(target_rectangle[0], target, 0.1);// adjust the size of car
      std::vector<cv::Point2f> four_points = target_rectangle[0].four_points;
      for(int i = 0; i < 8; ++i)
      {
        int j = i > 3 ? i - 4: i;
        float z = i > 3 ? seg_result.min_z: seg_result.max_z;
        cuboid[i].x = four_points[j].x;
        cuboid[i].y = four_points[j].y;
        cuboid[i].z = z;
      }
      geometry_msgs::Point t_point;
      t_point.x = center.x + 2 * cos(x_theta);
      t_point.y = center.y + 2 * sin(x_theta);
      float temp_dis[4];
      int mark[4]; //low mark
      //find nearst two points
      for (int i = 0; i < 4; ++i)
      {
        temp_dis[i] = (t_point.x - cuboid[i].x) * (t_point.x - cuboid[i].x) + (t_point.y - cuboid[i].y) * (t_point.y - cuboid[i].y);
        mark[i] = i;
      }
      // bubble sort
      for( int i = 0; i < 4; ++i)
      {
        for(int j = i + 1; j < 4; ++j)
        {
          if(temp_dis[i] > temp_dis[j])
          {
            float temp  = temp_dis[i];
            temp_dis[i] = temp_dis[j];
            temp_dis[j] = temp;
            int k   = mark[i];
            mark[i] = mark[j];
            mark[j] = k;
          }
        }
      }

      geometry_msgs::Point temp_point;
      temp_point.x =  0.5 * (cuboid[mark[0]].x + cuboid[mark[1]].x); // nearst two points
      temp_point.y =  0.5 * (cuboid[mark[0]].y + cuboid[mark[1]].y);
      cuboid[8].x = temp_point.x + 0.5 * cos(x_theta);
      cuboid[8].y = temp_point.y + 0.5 * sin(x_theta);
      cuboid[8].z = seg_result.max_z;
      cuboid[9]   = cuboid[8];
      cuboid[9].z = seg_result.min_z;
      pushBackCubePoints(cuboid_marker, cuboid);
      if(target.v_mean_absolute_ > v_threhold_)//m/s, draw the arrow
      {
        cuboid_marker.points.push_back(cuboid[mark[0]]);
        cuboid_marker.points.push_back(cuboid[8]);
        cuboid_marker.points.push_back(cuboid[mark[1]]);
        cuboid_marker.points.push_back(cuboid[8]);
        cuboid_marker.points.push_back(cuboid[mark[0] + 4]);
        cuboid_marker.points.push_back(cuboid[9]);
        cuboid_marker.points.push_back(cuboid[mark[1] + 4]);
        cuboid_marker.points.push_back(cuboid[9]);
        cuboid_marker.points.push_back(cuboid[9]);
        cuboid_marker.points.push_back(cuboid[8]);
      }
      break;
    }
    case Ped:
    case Unknow:
    {
      pushBackCubePoints(cuboid_marker, cuboid);
      break;
    }
    default:
    {
      std::cout << "Classify Error!" << std::endl;
      break;
    }
  }
}

void Drawing::pushBackCubePoints(visualization_msgs::Marker &cuboid_marker, geometry_msgs::Point cuboid[])
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

void Drawing::predictVehicleRect(rect_info &target_rectangle, Target head_target, float deta_x)
{
  std::cout << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
  cv::Point2f center = target_rectangle.center;
  std::vector<float> center_v;
  center_v.push_back(center.x);
  center_v.push_back(center.y);
  float v_angle = head_target.v_angle_in_own_xaxis_;
  float main_derection = target_rectangle.y_theta_mean;
  float length = target_rectangle.length;// main direction of length
  float width = target_rectangle.width;
  float angle_bias_threhold = 0.3;//10 degree
  float v_angle_bias_threhold = PI_/12;//10 degree
  float x_range = 30;
  float length_threhold =2.2;
  float width_threhold  =2.2;
  deta_x = head_target.vx_relative_;
  int is_slant = 1;   // slant or not
  int is_turning = 1; // turning or not
  int lw_unnormal = 0;
   float LW =0;
   float lw_rect = 0;
  if( head_target.length_ >length_threhold|| head_target.width_> 1.8)
  {
      LW  =  head_target.length_ /head_target.width_;
      lw_rect = head_target.rect_info_.length/head_target.rect_info_.width;
      if(LW <1.4 && LW > 0.7)
      {
          lw_unnormal = 1;
      }
      if( lw_rect >1.7 &&lw_rect <2.2)
      {
         lw_unnormal = 1;
      }
  }
  if((fabs(main_derection) < angle_bias_threhold) // slant or not
     ||(fabs(fabs(main_derection) - PI_) < angle_bias_threhold)
     ||(fabs(fabs(main_derection) - PI_/2) < angle_bias_threhold))
  {
    is_slant = 0;
  }
  // turning or not
  if(self_turning_ == false)
  {
      is_turning =0;
  }
  else
  {
       is_turning = 1;
  }

 if(is_turning == 0)
 {
       if( (fabs(fabs(v_angle) - PI_/2) > v_angle_bias_threhold &&fabs(fabs(v_angle) - PI_/2) < ( PI_/2 - v_angle_bias_threhold) )
           ||(head_target.rect_info_.width >= 1.6 && head_target.rect_info_.length >= 3.5) )
       {
         is_turning = 1;
       }
       if(is_turning == 1)
       {
           if(head_target.v_mean_absolute_ <= v_threhold_  )
           {
               is_turning == 0;
           }
       }
 }


  std::cout << " main_derection" << target_rectangle.y_theta << " v_angle:"<<v_angle << "v_x =" << deta_x<<" "<<head_target.vx_mean_relative_ <<"v"<<head_target.v_mean_absolute_<<std::endl;
  std::cout <<" is_slant=" << is_slant << " is_turning=" << is_turning << "self_turn="<<self_turning_ <<"LW"<<LW<< std::endl;
  std::vector<cv::Point2f> four_points = target_rectangle.four_points;
  cv::RotatedRect box_true_trans;
  enum REGION
  {
    REGION_1 = 1,
    REGION_2,
    REGION_3,
    REGION_4,
    REGION_5,
    REGION_6,
    REGION_7,
    REGION_8,
    REGION_9
  };
  int region = 0;
  if(head_target.classify_ == Car)//car
  {
    cv::Point2f rect[4];
    if(center.y >= 0)
    {
      if(center.x >= 3 && center.x < x_range)
      {
        region = REGION_3;
      }
      else if(center.x >= -3 && center.x < 3)
      {
        region = REGION_2;
      }
      else if(center.x > -x_range && center.x < -3 )
      {
        region = REGION_1;
        std::cout<<" locate:1"<<std::endl;
      }
    }
    else if(center.y < 0)
    {
      if(center.x >= 3 && center.x < x_range)
      {
        region = REGION_6;
      }
      else if(center.x >= -3 && center.x < 3)
      {
        region = REGION_5;
      }
      else if(center.x > -x_range && center.x < -3)
      {
        region = REGION_4;
      }
    }
    float length_standard = 4;
    float width_standard = 2;
    if(head_target.cube_.max_x - head_target.cube_.min_x >= 3.3)
    {
        length_standard = 2;
        width_standard = 4;
    }
    switch(region)
    {
      case REGION_1:
        rect[0].x = head_target.cube_.max_x - width_standard;         rect[0].y = head_target.cube_.min_y;
        break;
      case REGION_2:
        rect[0].x = head_target.cube_.min_x;            rect[0].y = head_target.cube_.min_y;
        break;
      case REGION_3:
        rect[0].x = head_target.cube_.min_x;           rect[0].y = head_target.cube_.min_y;
        break;
      case REGION_4:
        rect[0].x = head_target.cube_.max_x - width_standard;         rect[0].y = head_target.cube_.max_y - length_standard;
        break;
      case REGION_5:
        rect[0].x = head_target.cube_.min_x;            rect[0].y = head_target.cube_.max_y - length_standard;
        break;
      case REGION_6:
        rect[0].x = head_target.cube_.min_x;            rect[0].y = head_target.cube_.max_y - length_standard;
        break;
      default:
        break;
    }
    std::cout<<"region" <<region<<""<<rect[0].x<<" "<<rect[0].y<<std::endl;
    if(region)
    {
        rect[1].x =  rect[0].x + width_standard;             rect[1].y =  rect[0].y;
        rect[2].x =  rect[1].x;                   rect[2].y = rect[0].y + length_standard;
        rect[3].x =  rect[0].x;                   rect[3].y =  rect[2].y;
    }
    else
    {
        rect[1].x =  0;             rect[1].y =  0;
        rect[2].x =  0;             rect[2].y = 0;
        rect[3].x =  0;             rect[3].y = 0;
    }

    if(length < length_threhold && width < width_threhold)
    {
      target_rectangle.four_points.clear();
      std::cout << "car :" << (int)head_target.match_id_%1000 << "length<1.8 && width<1.8" << std::endl;
      for(int i = 0; i < 4; ++i)
      {
        target_rectangle.four_points.push_back(rect[i]);
      }
      target_rectangle.length = head_target.cube_.max_y - head_target.cube_.min_y;
      target_rectangle.width  = head_target.cube_.max_x - head_target.cube_.min_x;
      target_rectangle.y_theta = 0;
      main_derection = target_rectangle.y_theta ;
      length =  target_rectangle.length;
      width  = target_rectangle.width;
    }
    else
    {
      std::cout << "car :" << (int)head_target.match_id_%1000 << "    length>1.8 ||width > 1.8" << std::endl; 
      if( is_turning ==1)
      {
        std::cout<<"car :" << (int)head_target.match_id_%1000 << "This car is turning " << std::endl;
        float length_temp = length;
        float width_temp = width;
        if(length_temp < width_temp)
        {
          length = width_temp;
          width =length_temp;
          main_derection = main_derection +PI_/2;
        }
        float b = sinf(main_derection), a = cosf(main_derection);
        if (length / width < 1.3)
        {
          std::cout<<"This car need change length to 4 "<<std::endl;
          float dist_trans = (4.0 - length) / 2;// move the center along the main direction
          if ((center.x*(-b) + center.y*a) > 0)
          {
            center.x = center.x + dist_trans*(-b);
            center.y = center.y + dist_trans*a;
          }
          else
          {
            center.x = center.x - dist_trans*(-b);
            center.y = center.y - dist_trans*a;
          }
          box_true_trans = RotatedRect(Point2f(center.x, center.y), Size2f(2, 4), main_derection / PI_ * 180);
          target_rectangle.center = center;
          cv::Point2f rect[4];
          box_true_trans.points(rect);
          target_rectangle.four_points.clear();
          for(int i = 0; i < 4; ++i)
          {
            target_rectangle.four_points.push_back(rect[i]);
          }
        }
        else if(length / width> 3)
        {
          std::cout << "This car need change width to 2 " << std::endl;
          float dist_trans = (2 - width )/2;
          if((center.x * a + center.y * b) > 0)
          {
            center.x = center.x + dist_trans*a;
            center.y = center.y + dist_trans*b;
          }
          else
          {
            center.x = center.x - dist_trans*a;
            center.y = center.y - dist_trans*b;
          }
          box_true_trans = RotatedRect(Point2f(center.x, center.y), Size2f(2, 4), main_derection / PI_ * 180);
          target_rectangle.center = center;
          cv::Point2f rect[4];
          box_true_trans.points(rect);
          target_rectangle.four_points.clear();
          for(int i = 0; i < 4; ++i)
          {
            target_rectangle.four_points.push_back(rect[i]);
          }
        }
        else
        {
          std::cout << "  car hough line is ok   " << std::endl;
          target_rectangle.four_points = four_points;
        }
      }
      else if(is_turning == 0) // if the moving direction is along the y axis
      {
        std::cout << " car is moving  by y-axis   " << std::endl;
        target_rectangle.four_points.clear();
        for(int i = 0; i < 4; ++i)
        {
          target_rectangle.four_points.push_back(rect[i]);
        }
        target_rectangle.length = head_target.cube_.max_y - head_target.cube_.min_y;
        target_rectangle.width  = head_target.cube_.max_x - head_target.cube_.min_x;
        target_rectangle.y_theta = 0;
        main_derection = target_rectangle.y_theta;
        length = target_rectangle.length;
        width  = target_rectangle.width;
      }
      else
      {
          std::cout << "do nothing;" << std::endl;
      }
    }
  }

  else if(head_target.classify_ == Truck)//truck
  {
    cv::Point2f rect[4];
    if(center.y >= 0)
    {
      if(center.x >= 3 && center.x < x_range)
      {
        region = REGION_3;
      }
      else if(center.x >= -3 && center.x < 3)
      {
        region = REGION_2;
      }
      else if(center.x > -x_range && center.x < -3 )
      {
        region = REGION_1;
        std::cout<<" locate:1"<<std::endl;
      }
    }
    else if(center.y < 0)
    {
      if(center.x >= 3 && center.x < x_range)
      {
        region = REGION_6;
      }
      else if(center.x >= -3 && center.x < 3)
      {
        region = REGION_5;
      }
      else if(center.x > -x_range && center.x < -3)
      {
        region = REGION_4;
      }
    }

    float width_standard = 0;
    if(head_target.cube_.max_x - head_target.cube_.min_x <2.3)
    {
      width_standard = 2.8;
    } else
    {
      width_standard = head_target.cube_.max_x - head_target.cube_.min_x;
    }
    switch(region)
    {
      case REGION_1:
        rect[0].x = head_target.cube_.max_x - width_standard;         rect[0].y = head_target.cube_.min_y;
            break;
      case REGION_2:
        rect[0].x = head_target.cube_.min_x;            rect[0].y = head_target.cube_.min_y;
            break;
      case REGION_3:
        rect[0].x = head_target.cube_.min_x;           rect[0].y = head_target.cube_.min_y;
            break;
      case REGION_4:
        rect[0].x = head_target.cube_.max_x - width_standard;         rect[0].y = head_target.cube_.min_y;
            break;
      case REGION_5:
        rect[0].x = head_target.cube_.min_x;            rect[0].y = head_target.cube_.min_y;
            break;
      case REGION_6:
        rect[0].x = head_target.cube_.min_x;            rect[0].y = head_target.cube_.min_y;
            break;
      default:
        break;
    }
    std::cout<<"region" <<region<<""<<rect[0].x<<" "<<rect[0].y<<std::endl;
    if(region)
    {
      rect[1].x =  rect[0].x + width_standard;             rect[1].y =  rect[0].y;
      rect[2].x =  rect[1].x;                   rect[2].y = head_target.cube_.max_y;
      rect[3].x =  rect[0].x;                   rect[3].y = head_target.cube_.max_y;
    }
    else
    {
      rect[1].x =  0;             rect[1].y =  0;
      rect[2].x =  0;             rect[2].y = 0;
      rect[3].x =  0;             rect[3].y = 0;
    }


      if( is_turning ==1)
      {
        target_rectangle.four_points.clear();
        std::cout << "  car hough line is ok   " << std::endl;
        target_rectangle.four_points = four_points;

      }
      else if(is_turning == 0) // if the moving direction is along the y axis
      {
        std::cout << " car is moving  by y-axis   " << std::endl;
        target_rectangle.four_points.clear();
        for(int i = 0; i < 4; ++i)
        {
          target_rectangle.four_points.push_back(rect[i]);
        }
        target_rectangle.length = head_target.cube_.max_y - head_target.cube_.min_y;
        target_rectangle.width  = head_target.cube_.max_x - head_target.cube_.min_x;
        target_rectangle.y_theta = 0;
        main_derection = target_rectangle.y_theta;
        length = target_rectangle.length;
        width  = target_rectangle.width;
      }
      else
      {
        std::cout << "do nothing;" << std::endl;
      }

  }
}

void Drawing::selfTurning(const Tracking &tracking)
{
    if(tracking.yaw_.size() >10)
    {
        int last = tracking.yaw_.size() -1;
        if(tracking.yaw_[last] - tracking.yaw_[last -10] >= 0.1)
        {
            self_turning_ =1;
        }
        else {
            self_turning_ = 0;
        }
    }
}
}
