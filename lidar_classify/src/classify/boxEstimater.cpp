//
// Created by mjj on 17-4-20.
//

#include "boxEstimater.h"
namespace  Robosense{
void BoxEstimater::Rotated_point(vector<Point2f>  contours_temp, rect_info &target_rectangle,int rows,int cols)
{
    vector<Point2f>  validPts(contours_temp.size());
 #pragma omp parallel for schedule(dynamic)

    rect_info rect;
    Point2f rectpoint[4], rectpoint2[4], rectpoint3[4], rectpoint4[4],rectpoint5[2];
    vector<Point2f> contours_points;
    float scalar = 1,D_scalar=1,D_threshold=12;
    vector<float> result(2);
    result = HoughLines_first(contours_temp, 5, PI / 90,rows,cols);
    float b = sinf(result[0]), a = cosf(result[0]), c = result[1];
    vector<Point2f>::iterator contours_son = contours_temp.begin();
    for (; contours_son!=contours_temp.end(); contours_son++)
    {
        Point2f pts;
        pts.x = (*contours_son).x*a + (*contours_son).y*b;
        pts.y = (*contours_son).y*a - (*contours_son).x*b;
        contours_points.push_back(pts);
    }
    vector<Point2f>::iterator points_son = contours_points.begin();
    vector<float> x_position,y_position;
    float xmin, xmax, ymin, ymax;
    Point2f xy_min, xy_max, xy_min_true, xy_max_true;
    for (    ; points_son != contours_points.end(); points_son++)
    {
        x_position.push_back((*points_son).x);
        y_position.push_back((*points_son).y);
    }
    xmin = *min_element(x_position.begin(), x_position.end());
    xmax = *max_element(x_position.begin(), x_position.end());
    ymin = *min_element(y_position.begin(), y_position.end());
    ymax = *max_element(y_position.begin(), y_position.end());
    xy_min_true.x = xmin*a - ymin*b;
    xy_min_true.y = ymin*a + xmin*b;
    xy_max_true.x = xmax*a - ymax*b;
    xy_max_true.y = ymax*a + xmax*b;
    float D_width = xmax - xmin;
    float D_height = ymax - ymin;
    RotatedRect box_true_trans = RotatedRect(Point2f((xy_min_true.x + xy_max_true.x) / 2, (xy_min_true.y + xy_max_true.y)/2), Size2f(D_width,D_height),result[0]/PI*180);
    float x1 = box_true_trans.center.x;
    float y1 = box_true_trans.center.y;


    box_true_trans.points(rectpoint2);
    for (int j = 0; j < 4; j++)
    {
        rectpoint3[j].x = scalar*rectpoint2[j].x - (scalar - 1)*x1;
        rectpoint3[j].y = scalar*rectpoint2[j].y - (scalar - 1)*y1;
    }
    vector<Point2f> rectpoints2(rectpoint3, rectpoint3 + 4);

   rect.four_points = rectpoints2;
    rect.center.x = x1;
    rect.center.y = y1;
    rect.length = D_height;
    rect.width =D_width;

    if(result[0]<PI/2)
    {
       rect.main_derection =result[0]+PI/2   ;//(   0, PI); shun  shizhen ;
       rect.y_theta = result[0];
    }
    else
    {
      rect.y_theta = result[0] - PI ;
      rect.main_derection=result[0]-PI/2;
    }
    target_rectangle = rect;

}

void BoxEstimater::Rotated_point(vector<vector<Point2f> > contours_temp, vector<rect_info> &target_rectangle,int rows,int cols)
{
    vector<vector<Point2f> > validPts(contours_temp.size());
   #pragma omp parallel for schedule(dynamic)
    target_rectangle.clear();
    for (int i = 0; i < contours_temp.size(); i++)
    {
         rect_info rect;
        if (!contours_temp[i].empty())
        {
            Point2f rectpoint[4], rectpoint2[4], rectpoint3[4], rectpoint4[4],rectpoint5[2];
            vector<Point2f> contours_points;
            float scalar = 1,D_scalar=1,D_threshold=12;
            vector<float> result(2);
            result = HoughLines_first(contours_temp[i], 1, PI / 120,rows,cols);
            float b = sinf(result[0]), a = cosf(result[0]), c = result[1];
            vector<Point2f>::iterator contours_son = contours_temp[i].begin();
            for (; contours_son!=contours_temp[i].end(); contours_son++)
            {
                Point2f pts;
                pts.x = (*contours_son).x*a + (*contours_son).y*b;
                pts.y = (*contours_son).y*a - (*contours_son).x*b;
                contours_points.push_back(pts);
            }
            vector<Point2f>::iterator points_son = contours_points.begin();
            vector<float> x_position,y_position;
            float xmin, xmax, ymin, ymax;
            Point2f xy_min, xy_max, xy_min_true, xy_max_true;
            for (    ; points_son != contours_points.end(); points_son++)
            {
                x_position.push_back((*points_son).x);
                y_position.push_back((*points_son).y);
            }
            xmin = *min_element(x_position.begin(), x_position.end());
            xmax = *max_element(x_position.begin(), x_position.end());
            ymin = *min_element(y_position.begin(), y_position.end());
            ymax = *max_element(y_position.begin(), y_position.end());
            xy_min_true.x = xmin*a - ymin*b;
            xy_min_true.y = ymin*a + xmin*b;
            xy_max_true.x = xmax*a - ymax*b;
            xy_max_true.y = ymax*a + xmax*b;
            float D_width = xmax - xmin;
            float D_height = ymax - ymin;
            RotatedRect box_true_trans = RotatedRect(Point2f((xy_min_true.x + xy_max_true.x) / 2, (xy_min_true.y + xy_max_true.y)/2), Size2f(D_width,D_height),result[0]/PI*180);
            float x1 = box_true_trans.center.x;
            float y1 = box_true_trans.center.y;


            box_true_trans.points(rectpoint2);
            for (int j = 0; j < 4; j++)
            {
                rectpoint3[j].x = scalar*rectpoint2[j].x - (scalar - 1)*x1;
                rectpoint3[j].y = scalar*rectpoint2[j].y - (scalar - 1)*y1;
            }
            vector<Point2f> rectpoints2(rectpoint3, rectpoint3 + 4);

           rect.four_points = rectpoints2;
            rect.center.x = x1;
            rect.center.y = y1;
//            rect.length = D_height > D_width ? D_height : D_width;
//            rect.width = D_height < D_width ? D_height : D_width;
            rect.length = D_height;
            rect.width =D_width;

            if(result[0]<PI/2)
            {
              rect.main_derection =result[0]+PI/2   ;//(   0, PI); shun  shizhen ;
              rect.y_theta = result[0];
            }
            else
            {
                 rect.y_theta = result[0] - PI ;
                 rect.main_derection=result[0]-PI/2;
            }
          //(0,PI),主方向与y轴的夹角
        }
     target_rectangle.push_back(rect);
    }
}


vector<float> BoxEstimater::HoughLines_first(vector<Point2f> it, float rho, float theta,int rows,int cols)
{
    vector<Point2f>::iterator it_son = it.begin();
    AutoBuffer<int> _accum, _sort_buf;
    AutoBuffer<float>_tabSin, _tabCos;
    int i, j, temp = 0, idx = 0;
    int width, height;
    int numangle, numrho;
    int r, n;
    float irho = 1 / rho, result_rho, result_angle;
    double scale;
    vector<float> result(2);
    width = cols;
    height = rows;
    numangle = cvRound(PI / theta);
    numrho = cvRound(((width + height) * 2 + 1) / rho);
    _accum.allocate((numangle + 2)*(numrho + 2));
    _sort_buf.allocate(numangle*numrho);
    _tabSin.allocate(numangle);
    _tabCos.allocate(numangle);
    int *accum = _accum, *sort_buf = _sort_buf;
    float *tabSin = _tabSin, *tabCos = _tabCos;
    memset(accum, 0, sizeof(accum[0])*(numangle + 2)*(numrho + 2));

    for (int i = 0; i < numangle; ++i)
    {
        tabSin[i] = (float)(sinf(i*theta)*irho);
        tabCos[i] = (float)(cosf(i*theta)*irho);
    }

    for (; it_son != it.end(); it_son++)
    {
        (*it_son).x=10*(*it_son).x;
        (*it_son).y=10*(*it_son).y;
        for (n = 0; n < numangle; n++)
        {
            r = cvRound((*it_son).x*tabCos[n] + (*it_son).y*tabSin[n]);
            r += (numrho - 1) / 2;
            accum[(n + 1)*(numrho + 2) + r + 1]++;
        }
    }
    int account = (numangle + 2)*(numrho + 2);
    for (i = 0; i<account; i++)
    {
        if (temp < accum[i])
        {
            temp = accum[i];
            idx = i;
        }
    }
    scale = 1. / (numrho + 2);
    n = cvFloor(idx*scale) - 1;
    result_angle = n*theta;
    r = idx - (n + 1)*(numrho + 2) - 1;
    result_rho = (r - (numrho - 1)*0.5f)*rho;
    result[0]=result_angle;
    result[1]=result_rho/10;
    return result;
}
}
