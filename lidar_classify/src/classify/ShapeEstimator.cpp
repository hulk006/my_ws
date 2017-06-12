#include "ShapeEstimator.h"
#include "stack"

ShapeEstimator::ShapeEstimator(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_)
{
    test = 0;
    halfcloud = ptr_->width;
//    std::cout<<"pointsNum:"<<labeled_pc.points.size()<<std::endl;
//    std::cout<<"x,y,z:"<<labeled_pc.points[0].x<<" "<<labeled_pc.points[0].y<<" "<<labeled_pc.points[0].z<<std::endl;
    //seg_num = label_num;
    //max_z.resize(label_num+1);
    //min_z.resize(label_num+1);
    //max_axis.resize(label_num+1);
    //min_axis.resize(label_num+1);
    //four_p.resize(label_num+1);
    //maindirect.resize(label_num+1);ptr_
    DISTHREHOLD = 0.2 * 0.2;
    nearsetPoint_P = findNearest_P(ptr_);

//    std::cout<<"nearsetPoint_P:"<<nearsetPoint_P.size()<<std::endl;

    convertedCVPoint_P = convert2CVPoint_P(ptr_);
    cv::Point2f lz = findLimit_Z(ptr_);
    if(nearsetPoint_P.size() > 0)
    {
        mainDirect_P =  findMainDirect_RANSAC_P(convertedCVPoint_P);
    }
    else
    {
        mainDirect_P.x = 1;
        mainDirect_P.y = 0;
    }

    box_P =  findBox_P(convertedCVPoint_P, mainDirect_P);
    //four_p[seg] = boxp;
    max_z_P = lz.x;
    min_z_P = lz.y;

}

float ShapeEstimator::distance(pcl::PointXYZI p)
{return p.x * p.x + p.y * p.y;}

void ShapeEstimator::testnearestpoint(visualization_msgs::MarkerArray& mkarry)
{
    visualization_msgs::Marker mk;
    mk.header.frame_id = "velodyne";
    mk.id = 0;
    mk.ns = "testmk";
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.r = 0;
    mk.color.g = 1;
    mk.color.b = 0;
    mk.color.a = 1;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 0;

    for(int seg = 1; seg < ob.size(); ++seg)
    {
        //if(test){std::cout << "seg:  "<< nearestpoint.size()<<std::endl;}
        //mk.points.clear();
        for(int pp = 0; pp <ob[seg].size();++pp )
        {
            //point_d tmp = nearestpoint[seg][pp];
            //if(test){std::cout<<"col:" << tmp.col << std::endl;}
            cv::Point2f tmp = ob[seg][pp];
            geometry_msgs::Point p1;
            p1.x = tmp.x;
            p1.y = tmp.y;
            p1.z = 1.5;
            mk.points.push_back(p1);
        }
    }
    mkarry.markers.push_back(mk);
    ///////////////////////////////////////////////////////////////////////////////////
    //if(test){std::cout << "mk_msg:  "<< mk.markers.size()<<std::endl;}
    mk.id = 1;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1;
    mk.color.g = 0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 0;
    mk.points.clear();

    for(int seg = 1; seg < line_seg.size(); ++seg){
        std::vector<std::vector<cv::Point2f> > tmp_par = line_seg[seg];
        for(int part = 0; part < tmp_par.size();++part){
            std::vector<cv::Point2f>  tmp_line = tmp_par[part];
            geometry_msgs::Point p;
            p.x = tmp_line[0].x;    p.y = tmp_line[0].y;    p.z = 2;
            mk.points.push_back(p);
            p.x = tmp_line[tmp_line.size()-1].x;    p.y = tmp_line[tmp_line.size()-1].y;    p.z = 2;
            mk.points.push_back(p);
        }
    }
    //mkarry.markers.push_back(mk);
    ////////////////////////////////////////////////////////////////////////////////////
    mk.id = 2;
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1;
    mk.color.g = 0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 0;
    mk.points.clear();

    for(int seg = 1; seg < max_axis.size(); seg ++){
        std::vector<geometry_msgs::Point> b(9);
        //if(test){std::cout<<max_z[seg] - min_z[seg]<<std::endl;}
        float size_b =(four_p[seg][0].x - four_p[seg][1].x) * (four_p[seg][0].x - four_p[seg][1].x) +
                              (four_p[seg][0].y - four_p[seg][1].y)*(four_p[seg][0].y - four_p[seg][1].y) +
                              (four_p[seg][1].x - four_p[seg][2].x) * (four_p[seg][1].x - four_p[seg][2].x) +
                              (four_p[seg][1].y - four_p[seg][2].y)*(four_p[seg][1].y - four_p[seg][2].y);
        if(size_b > (20*20)){continue;}

        b[0].x = four_p[seg][0].x;
        b[0].y = four_p[seg][0].y;
        b[0].z = max_z[seg];
        b[1].x = four_p[seg][1].x;
        b[1].y = four_p[seg][1].y;
        b[1].z = max_z[seg];
        b[2].x = four_p[seg][2].x;
        b[2].y = four_p[seg][2].y;
        b[2].z = max_z[seg];
        b[3].x = four_p[seg][3].x;
        b[3].y = four_p[seg][3].y;
        b[3].z = max_z[seg];
        mk.points.push_back(b[0]);  mk.points.push_back(b[1]);
        mk.points.push_back(b[1]);  mk.points.push_back(b[2]);
        mk.points.push_back(b[2]);  mk.points.push_back(b[3]);
        mk.points.push_back(b[3]);  mk.points.push_back(b[0]);
        b[4].x = four_p[seg][0].x;
        b[4].y = four_p[seg][0].y;
        b[4].z = min_z[seg];
        b[5].x = four_p[seg][1].x;
        b[5].y = four_p[seg][1].y;
        b[5].z = min_z[seg];
        b[6].x = four_p[seg][2].x;
        b[6].y = four_p[seg][2].y;
        b[6].z = min_z[seg];
        b[7].x = four_p[seg][3].x;
        b[7].y = four_p[seg][3].y;
        b[7].z = min_z[seg];
        mk.points.push_back(b[4]);  mk.points.push_back(b[5]);
        mk.points.push_back(b[5]);  mk.points.push_back(b[6]);
        mk.points.push_back(b[6]);  mk.points.push_back(b[7]);
        mk.points.push_back(b[7]);  mk.points.push_back(b[4]);

        mk.points.push_back(b[0]);  mk.points.push_back(b[4]);
        mk.points.push_back(b[1]);  mk.points.push_back(b[5]);
        mk.points.push_back(b[2]);  mk.points.push_back(b[6]);
        mk.points.push_back(b[3]);  mk.points.push_back(b[7]);
    }
    mkarry.markers.push_back(mk);
}

cv::Point2f ShapeEstimator::trans(cv::Point2f p, float theta)
{
    cv::Point2f tp;
    float cos_ = cos(theta);
    float sin_ = sin(theta);
    tp.x = cos_ * p.x + sin_ * p.y;
    tp.y = cos_ * p.y - sin_ * p.x;
    return tp;
}

cv::Point2f ShapeEstimator::retrans(cv::Point2f p, float theta)
{
    cv::Point2f tp;
    float cos_ = cos(theta);
    float sin_ = sin(theta);
    tp.x = cos_ * p.x - sin_ * p.y;
    tp.y = cos_ * p.y + sin_ * p.x;
    return tp;
}

cv::Point2f ShapeEstimator::findMainDirect_RANSAC_P(std::vector<cv::Point2f> in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>);
    tmp_pc->height = 1;
    for(int pp = 0; pp < in.size();++pp){
        pcl::PointXYZ p ;
        p.x = in[pp].x;
        p.y = in[pp].y;
        p.z = 2;
        tmp_pc->points.push_back(p);
        ++tmp_pc->width;
    }//end for
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg1;
    seg1.setOptimizeCoefficients(true);
    seg1.setModelType(pcl::SACMODEL_LINE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setDistanceThreshold(0.1);
    seg1.setInputCloud(tmp_pc);
    seg1.segment(*inliers, *coefficients);//[point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
    cv::Point2f direct;
    direct.x = coefficients->values[3];
    direct.y = coefficients->values[4];
    return direct;
}

std::vector<cv::Point2f> ShapeEstimator::findBox_P(std::vector<cv::Point2f> in, cv::Point2f direct)
{
    float theta = atan2(direct.y,direct.x);
//    std::cout <<theta<<std::endl;
    cv::Point2f limit_x;//x = max  y = min
    cv::Point2f limit_y;
    //std::cout <<"in.size():" << in.size()<<std::endl;
    //std::cout <<"in[0].y:" << in[0].y<<std::endl;
    limit_x.x = (trans(in[0], theta).x);    limit_x.y = (trans(in[0], theta).x);
    limit_y.x = (trans(in[0], theta).y);    limit_y.y = (trans(in[0], theta).y);
    for(int pp = 0; pp < in.size();++pp){
        cv::Point2f tp = trans(in[pp], theta);
        limit_x.x = tp.x > limit_x.x? tp.x:limit_x.x;
        limit_x.y = tp.x < limit_x.y? tp.x:limit_x.y;
        limit_y.x = tp.y > limit_y.x? tp.y:limit_y.x;
        limit_y.y = tp.y < limit_y.y? tp.y:limit_y.y;
    }
    std::vector<cv::Point2f> point4;
    cv::Point2f tp;
    tp.x = limit_x.x;
    tp.y = limit_y.x;
    tp = retrans(tp,theta);
    point4.push_back(tp);
    tp.x = limit_x.x;
    tp.y = limit_y.y;
    tp = retrans(tp,theta);
    point4.push_back(tp);
    tp.x = limit_x.y;
    tp.y = limit_y.y;
    tp = retrans(tp,theta);
    point4.push_back(tp);
    tp.x = limit_x.y;
    tp.y = limit_y.x;
    tp = retrans(tp,theta);
    point4.push_back(tp);
    return point4;

}

std::vector<cv::Point2f> ShapeEstimator::convert2CVPoint_P(pcl::PointCloud<pcl::PointXYZI>::Ptr in)
{
    std::vector<cv::Point2f> tmp_;
    for(int pp = 0; pp < in->points.size(); ++pp){
        cv::Point2f tp;
        tp.x = in->points[pp].x;
        tp.y = in->points[pp].y;
        tmp_.push_back(tp);
    }
    return tmp_;
}

std::vector<cv::Point2f> ShapeEstimator::findNearest_P(pcl::PointCloud<pcl::PointXYZI>::Ptr in)
{
    std::vector<cv::Point2f> tmp_(1800);//0.2
    std::vector<float> dis_(1800, 0);
    for(int pp = 0; pp < in->points.size(); ++pp){
        pcl::PointXYZI tp = in->points[pp];
        cv::Point2f tcp;
        tcp.x = tp.x;
        tcp.y = tp.y;
        float theta = atan2(tp.y,tp.x);
        theta+=M_PI;
        float disp = distance(tp);
        //std::cout<<"theta: "<<theta;
        int block = (int) (theta / (0.2 * M_PI/180));
        //std::cout<<"block:"<<block<<std::endl;
        if(dis_[block] == 0){
            tmp_[block] = tcp;
            dis_[block] = disp;
        }else if(disp < dis_[block]){
            tmp_[block] = tcp;
            dis_[block] = disp;
        }
    }//end for
    std::vector<cv::Point2f> out;
    for(int block = 0; block < tmp_.size();block++){
        if(dis_[block] != 0){
            out.push_back(tmp_[block]);
        }
    }
    return out;
}

cv::Point2f ShapeEstimator::findLimit_Z(pcl::PointCloud<pcl::PointXYZI>::Ptr in)
{
    cv::Point2f out;
    out.x = in->points[0].z;//max
    out.y = in->points[0].z;//min
    for(int pp = 0; pp < in->points.size();++pp){
        out.x = out.x > in->points[pp].z ? out.x : in->points[pp].z;
        out.y = out.y < in->points[pp].z ? out.y : in->points[pp].z;
    }
    return out;
}
