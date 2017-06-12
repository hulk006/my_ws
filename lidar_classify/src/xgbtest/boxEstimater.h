//
// Created by mjj on 17-4-20. 
//

#ifndef LIDAR_SEG_MULTISCALE_CAR_CUBE_H
#define LIDAR_SEG_MULTISCALE_CAR_CUBE_H

#include <opencv2/opencv.hpp>

namespace Robosense {
using namespace std;
using namespace cv;
struct rect_info
{
    cv::Point2f center;
    vector<cv::Point2f> four_points;
    float width;
    float lenth;
    float main_derection;
    float y_seta;
    float y_seta_mean;
};


class BoxEstimater
{
private:
    int rows;
    int cols;
    float PI;
public:

    BoxEstimater(){

        PI = 3.1415926;
    }
    ~BoxEstimater(){}
    void Rotated_point(vector<vector<cv::Point2f> > contours_temp, vector<rect_info> &target_rectangle,int rows, int cols);
    vector<float> HoughLines_first(vector<cv::Point2f>, float rho, float theta,int rows, int cols);
    void Rotated_point(vector<cv::Point2f>  contours_temp, rect_info &target_rectangle,int rows,int cols);
};
}
#endif //LIDAR_SEG_MULTISCALE_CAR_CUBE_H
