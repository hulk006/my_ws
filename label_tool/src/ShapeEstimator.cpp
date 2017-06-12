#include "ShapeEstimator.h"
#include "stack"

/*ShapeEstimator::ShapeEstimator(pcl::PointCloud<pcl::PointXYZL> labeled_pc, int label_num)
{
    test = 0;
    halfcloud = labeled_pc.width;
    seg_num = label_num;
    max_z.resize(label_num+1);
    min_z.resize(label_num+1);
    max_axis.resize(label_num+1);
    min_axis.resize(label_num+1);
    four_p.resize(label_num+1);
    maindirect.resize(label_num+1);
    DISTHREHOLD = 0.2 * 0.2;
    if(test)
    {
        std::cout<< "ShapeEstimatorb" <<std::endl;
        std::cout<<"Width: " <<  labeled_pc.width << "Heigth: " << labeled_pc.height<<std::endl;
    }
    nearPoint(labeled_pc,label_num);
    if(test){std::cout<< "rerange..." <<std::endl;}
    rerange();
    if(test){std::cout<< "ShapeEstimatore" <<std::endl;}
    IEPF();
    //findBox();
    if(test){std::cout<< "findmaindirect" <<std::endl;}
    findmaindirect();
    //covfindmaindirect();
    if(test){std::cout<< "findBoxRANSAC" <<std::endl;}
    findBoxRANSAC();

}*/

ShapeEstimator::ShapeEstimator(pcl::PointCloud<pcl::PointXYZL> labeled_pc, int label_num)
{
    test = 1;
    halfcloud = labeled_pc.width;
    seg_num = label_num;
    max_z.resize(label_num+1);
    min_z.resize(label_num+1);
    max_axis.resize(label_num+1);
    min_axis.resize(label_num+1);
    four_p.resize(label_num+1);
    maindirect.resize(label_num+1);
    DISTHREHOLD = 0.2 * 0.2;


    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr > segpc;
    segpc = pointCloudSeparate(labeled_pc,label_num);

    for(int seg = 1; seg < segpc.size(); ++seg){
        std::cout << "seg:"<< seg<<std::endl;
        std::vector<cv::Point2f> np = findNearest_P(segpc[seg]);
        std::cout << "np:"<< np.size()<<std::endl;
        std::vector<cv::Point2f> cp = convert2CVPoint_P(segpc[seg]);
        std::cout << "cp:"<< cp.size()<<std::endl;
        cv::Point2f lz = findLimit_Z(segpc[seg]);
        cv::Point2f dirp =  findMainDirect_RANSAC_P(cp);
        std::vector<cv::Point2f> boxp =  findBox_P(cp, dirp);
        four_p[seg] = boxp;
        max_z[seg] = lz.x;
        min_z[seg] = lz.y;
    }
}

ShapeEstimator::ShapeEstimator(pcl::PointCloud<pcl::PointXYZL> labeled_pc)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr ptr_(new pcl::PointCloud<pcl::PointXYZL>);
    *ptr_ = labeled_pc;
    test = 0;
    halfcloud = labeled_pc.width;
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

void ShapeEstimator::nearPoint(pcl::PointCloud<pcl::PointXYZL> labeled_pc, int label_num)
{
    std::vector<point_d> t123;
    std::vector<cv::Point2f> yy;
    nearestpoint.push_back(t123);
    seg_point.push_back(yy);//first not use

    for(int seg = 1; seg <= label_num; ++seg)
    {
        //if(test){std::cout <<"Seg : "<< seg << std::endl;}

        std::vector<point_d> tmp_arry;
        std::vector<cv::Point2f> tmp_cvpoint;
        for(int col = 0; col < labeled_pc.width; ++ col)
        {
            //if(test){std::cout<<"row : " << row << std::endl;}

            int first_mk = 1;
            point_d tmp_;
            for(int row = 0; row < labeled_pc.height; ++row)
            {
                //if(test){std::cout <<"col : "<< col << std::endl;}

                if(labeled_pc.points[row * labeled_pc.width + col].label == seg)
                {
                    cv::Point2f c_p;
                    c_p.x = labeled_pc.points[row * labeled_pc.width + col].x;
                    c_p.y = labeled_pc.points[row * labeled_pc.width + col].y;
                    tmp_cvpoint.push_back(c_p);
                    if(first_mk)
                    {
                        tmp_.col = col;
                        tmp_.p_ = labeled_pc.points[row * labeled_pc.width + col];
                        tmp_.dis_ = distance(tmp_.p_);
                        //max_z[seg] = labeled_pc.points[row * labeled_pc.width + col].z;
                        //min_z[seg] = labeled_pc.points[row * labeled_pc.width + col].z;
                        first_mk = 0;
                    }else
                    {
                        pcl::PointXYZL test_p = labeled_pc.points[row * labeled_pc.width + col];
                        //max_z[seg] = test_p.z > max_z[seg] ? test_p.z:max_z[seg];
                        //min_z[seg] = test_p.z < min_z[seg] ? test_p.z:min_z[seg];
                        if(distance(test_p) < distance(tmp_.p_))
                        {
                            tmp_.dis_ = distance(test_p) ;
                            tmp_.p_ = test_p;
                        }
                    }//end first_mk
                }
            }//end row
            if(!first_mk){tmp_arry.push_back(tmp_);}
        }//end col
        seg_point.push_back(tmp_cvpoint);
        nearestpoint.push_back(tmp_arry);
    }//end seg

    std::vector<int > first_mks(label_num+1,1);
    for(int col = 0; col < labeled_pc.width; ++ col)
    {
        for(int row = 0; row < labeled_pc.height; ++row)
        {
            if(labeled_pc.points[row * labeled_pc.width + col].label > 0)
            {
                int lab = labeled_pc.points[row * labeled_pc.width + col].label;
                if(first_mks[lab])
                {
                    max_z[lab] = labeled_pc.points[row * labeled_pc.width + col].z;
                    min_z[lab] = labeled_pc.points[row * labeled_pc.width + col].z;
                    first_mks[lab] = 0;
                }else
                {
                    pcl::PointXYZL test_p = labeled_pc.points[row * labeled_pc.width + col];
                    max_z[lab] = test_p.z > max_z[lab] ? test_p.z:max_z[lab];
                    min_z[lab] = test_p.z < min_z[lab] ? test_p.z:min_z[lab];
                }
            }
        }
    }//end col

}

float ShapeEstimator::distance(pcl::PointXYZL p)
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

void ShapeEstimator::rerange()
{
    //nearestpoint;
    for(int seg = 1; seg < nearestpoint.size(); ++ seg )
    {
        int mk = 0;
        int breakp = 0;
        std::vector<point_d> tmp = nearestpoint[seg];
        //if(test){std::cout << (nearestpoint[seg].size()-1)<<std::endl;}
        for(int pp = 0; pp < (tmp.size()-1);++pp)
        {
            //if(test){std::cout << "123"<<std::endl;}
            if((tmp[pp+1].col - tmp[pp].col) > halfcloud)
            {
                mk = 1;
                breakp = pp;
                break;
            }
        }//end perpoint
        //if(test){std::cout << "a??"<<std::endl;}
        if(mk)
        {
            //if(test){std::cout << "a?????????"<<std::endl;}
            mk = 0;
            int index = 0;
            for(int bk = (breakp+ 1); bk < tmp.size(); ++bk, ++index)
            {
                nearestpoint[seg][index].dis_ = tmp[bk].dis_;
                nearestpoint[seg][index].col = tmp[bk].col;
                nearestpoint[seg][index].p_ = tmp[bk].p_;
            }
            for(int bk =0; bk <  (breakp+ 1); ++bk, ++index)
            {
                nearestpoint[seg][index].dis_ = tmp[bk].dis_;
                nearestpoint[seg][index].col = tmp[bk].col;
                nearestpoint[seg][index].p_ = tmp[bk].p_;
            }
        }
    }//end seg
}

void ShapeEstimator::IEPF()
{

    std::vector<cv::Point2f> ttt;
    ob.push_back(ttt);
    //convert to cv point
    for(int seg = 1; seg < nearestpoint.size(); ++seg)
    {
        std::vector<cv::Point2f> tmparry;
        for(int pp = 0; pp < nearestpoint[seg].size();++pp)
        {
            cv::Point2f tmp_p;
            tmp_p.x = nearestpoint[seg][pp].p_.x;
            tmp_p.y = nearestpoint[seg][pp].p_.y;
            tmparry.push_back(tmp_p);
        }//end point
        ob.push_back(tmparry);
    }//end seg


    std::vector<std::vector<cv::Point2f> > tt;
    line_seg.push_back(tt);//first is not used
    for(int seg = 1; seg < ob.size(); ++seg){
        std::vector<std::vector<cv::Point2f> > result;
        std::stack<std::vector<cv::Point2f> > parts;
        parts.push(ob[seg]);
        while(!parts.empty()){
            std::vector<cv::Point2f> tmp = parts.top();
            parts.pop();
            if(tmp.size() <= 2){
                result.push_back(tmp);
            }else{
                cv::Point2f p1 = tmp[0];
                cv::Point2f p2 = tmp[tmp.size() - 1];
                // 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
                // 化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
                // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
                float A,B,C;
                A  = p2.y - p1.y;   B = p1.x - p2.x;    C = p2.x*p1.y - p1.x * p2.y;
                int break_idx = 0;
                float max_dis = 0;
                for(int pp = 0;pp < (tmp.size());++pp){
                    cv::Point2f tmp_p = tmp[pp];
                    float dis = ((A * tmp_p.x + B * tmp_p.y + C) * (A * tmp_p.x + B * tmp_p.y + C) )/(A*A + B*B);
                    if(dis > max_dis){
                        max_dis = dis;
                        break_idx = pp;
                    }
                }//find max dis
                if(max_dis > (DISTHREHOLD)){
                    std::vector<cv::Point2f> aaa;
                    for(int pp = 0 ; pp <= break_idx; ++pp){aaa.push_back(tmp[pp]);}
                    parts.push(aaa);
                    aaa.clear();
                    for(int pp = break_idx; pp < tmp.size(); ++pp){aaa.push_back(tmp[pp]);}
                    parts.push(aaa);
                }else{
                    result.push_back(tmp);
                }
            }//end if
        }//end while
        line_seg.push_back(result);
    }//end seg for
}

void ShapeEstimator::findBox()
{
    maindirect.resize(ob.size());
    std::vector<std::vector<cv::Point2f> > max_part_seg(ob.size());

    //if(test){std::cout << "1"<<std::endl;}
    for(int seg = 1; seg < line_seg.size(); ++seg){
        std::vector<std::vector<cv::Point2f> > tmp_seg = line_seg[seg];
        int max_part = 0;
        for(int pa = 0; pa < tmp_seg.size(); ++ pa){
            if(tmp_seg[pa].size() > tmp_seg[max_part].size()){ max_part = pa;}
        }
        max_part_seg[seg] = tmp_seg[max_part];
    }//end for //find max part for each seg

    //if(test){std::cout << "2"<<std::endl;}
    std::vector<cv::Point2f > line_seg(ob.size());//x=A y=B
    for(int seg = 1; seg < line_seg.size();++seg){
        cv::Point2f tp;
        cv::Point2f p1 = max_part_seg[seg][0];
        cv::Point2f p2 = max_part_seg[seg][max_part_seg[seg].size()-1];
        // 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
        // 化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
        // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
        float A,B,C;
        A  = p2.y - p1.y;   B = p1.x - p2.x;    C = p2.x*p1.y - p1.x * p2.y;
        line_seg[seg].x = A;
        line_seg[seg].y = B;
        //if(test){std::cout << seg<<std::endl;}
    }//A  B

    //if(test){std::cout << "3"<<std::endl;}
    std::vector<float > theta(ob.size());
    for(int seg = 1; seg < theta.size();++seg){
        if(line_seg[seg].x == 0){
            theta[seg] = 0;
        }else if(line_seg[seg].y == 0 ){
            theta[seg] = M_PI/2;
        }else{
            theta[seg] = atan2(-line_seg[seg].x/line_seg[seg].y,1);
            //if(test){std::cout<<seg<<":"<<theta[seg]<<std::endl;}
        }
    }//theta

    //if(test){std::cout << "4"<<std::endl;}
    std::vector<cv::Point2f> limit_x(ob.size());//x = max  y = min
    std::vector<cv::Point2f> limit_y(ob.size());
    for(int seg = 1; seg < ob.size(); ++ seg){
        std::vector<cv::Point2f> tmp_seg = ob[seg];
        limit_x[seg].x = (trans(tmp_seg[0], theta[seg]).x);
        //if(test){std::cout<<seg<<":"<< limit_x[seg].x<<std::endl;}
        limit_x[seg].y = (trans(tmp_seg[0], theta[seg]).x);
        limit_y[seg].x = (trans(tmp_seg[0], theta[seg]).y);
        limit_y[seg].y = (trans(tmp_seg[0], theta[seg]).y);
        for(int pp = 0; pp < tmp_seg.size();++pp){
            cv::Point2f tp = trans(tmp_seg[pp], theta[seg]);
            limit_x[seg].x = tp.x > limit_x[seg].x? tp.x:limit_x[seg].x;
            limit_x[seg].y = tp.x < limit_x[seg].y? tp.x:limit_x[seg].y;
            limit_y[seg].x = tp.y > limit_y[seg].x? tp.y:limit_y[seg].x;
            limit_y[seg].y = tp.y < limit_y[seg].y? tp.y:limit_y[seg].y;
        }

    }

    //if(test){std::cout << "5"<<std::endl;}
    std::vector<cv::Point2f> max_(ob.size());
    std::vector<cv::Point2f> min_(ob.size());
    for(int seg = 1; seg<ob.size();++seg)
    {
        cv::Point2f tp;
        tp.x = limit_x[seg].x;
        tp.y = limit_y[seg].x;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
        tp.x = limit_x[seg].x;
        tp.y = limit_y[seg].y;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
        tp.x = limit_x[seg].y;
        tp.y = limit_y[seg].y;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
        tp.x = limit_x[seg].y;
        tp.y = limit_y[seg].x;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
    }

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

void ShapeEstimator::findmaindirect()
{
    for(int seg = 1; seg < ob.size();++seg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_pc->height = 1;
        std::vector<cv::Point2f> tmp_cvp = ob[seg];
        for(int pp = 0; pp < ob[seg].size();++pp){

            pcl::PointXYZ p ;
            p.x = tmp_cvp[pp].x;
            p.y = tmp_cvp[pp].y;
            p.z = 2;
            tmp_pc->points.push_back(p);
            ++tmp_pc->width;
        }//point cloud get
        //if(test){std::cout<<seg<<":"<<ob.size()<<std::endl;}
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg1;
        seg1.setOptimizeCoefficients(true);
        seg1.setModelType(pcl::SACMODEL_LINE);
        seg1.setMethodType(pcl::SAC_RANSAC);
        seg1.setDistanceThreshold(0.1);
        seg1.setInputCloud(tmp_pc);
        seg1.segment(*inliers, *coefficients);//[point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
        maindirect[seg].x = coefficients->values[3];
        maindirect[seg].y = coefficients->values[4];
    }
}

void ShapeEstimator::findBoxRANSAC()
{
    std::vector<float > theta(ob.size());
    for(int seg = 1; seg < theta.size();++seg){
        theta[seg] = atan2(maindirect[seg].y,maindirect[seg].x);
    }//theta

    //if(test){std::cout << "4"<<std::endl;}
    std::vector<cv::Point2f> limit_x(ob.size());//x = max  y = min
    std::vector<cv::Point2f> limit_y(ob.size());
    for(int seg = 1; seg < ob.size(); ++ seg){
        std::vector<cv::Point2f> tmp_seg = seg_point[seg];
        limit_x[seg].x = (trans(tmp_seg[0], theta[seg]).x);
        //if(test){std::cout<<seg<<":"<< limit_x[seg].x<<std::endl;}
        limit_x[seg].y = (trans(tmp_seg[0], theta[seg]).x);
        limit_y[seg].x = (trans(tmp_seg[0], theta[seg]).y);
        limit_y[seg].y = (trans(tmp_seg[0], theta[seg]).y);
        for(int pp = 0; pp < tmp_seg.size();++pp){
            cv::Point2f tp = trans(tmp_seg[pp], theta[seg]);
            limit_x[seg].x = tp.x > limit_x[seg].x? tp.x:limit_x[seg].x;
            limit_x[seg].y = tp.x < limit_x[seg].y? tp.x:limit_x[seg].y;
            limit_y[seg].x = tp.y > limit_y[seg].x? tp.y:limit_y[seg].x;
            limit_y[seg].y = tp.y < limit_y[seg].y? tp.y:limit_y[seg].y;
        }

    }

    //if(test){std::cout << "5"<<std::endl;}
    std::vector<cv::Point2f> max_(ob.size());
    std::vector<cv::Point2f> min_(ob.size());
    for(int seg = 1; seg<ob.size();++seg)
    {
        cv::Point2f tp;
        tp.x = limit_x[seg].x;
        tp.y = limit_y[seg].x;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
        tp.x = limit_x[seg].x;
        tp.y = limit_y[seg].y;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
        tp.x = limit_x[seg].y;
        tp.y = limit_y[seg].y;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
        tp.x = limit_x[seg].y;
        tp.y = limit_y[seg].x;
        tp = retrans(tp,theta[seg]);
        four_p[seg].push_back(tp);
    }
}

void ShapeEstimator::covfindmaindirect()
{
    for(int seg = 1; seg < seg_point.size();++seg){
        std::vector<cv::Point2f> tmp1 = seg_point[seg];
        float x_avr = 0;    float y_avr = 0;
        for(int pp = 0; pp < tmp1.size();++pp ){
            x_avr += tmp1[pp].x;
            y_avr += tmp1[pp].y;
        }
        x_avr /= (tmp1.size()-1);
        y_avr /= (tmp1.size()-1);//find avr
        float cov_mat[3] = {0};
        for(int pp = 0; pp < tmp1.size();++pp){
            cov_mat[0] += (tmp1[pp].x - x_avr) * (tmp1[pp].x - x_avr);//cov(x,x)
            cov_mat[1] += (tmp1[pp].x - x_avr) * (tmp1[pp].y - y_avr);//cov(x,y)
            cov_mat[2] += (tmp1[pp].y - y_avr) * (tmp1[pp].y - y_avr);//cov(y,y)
        }
        cov_mat[0] /= (tmp1.size() - 1);
        cov_mat[1] /= (tmp1.size() - 1);
        cov_mat[2] /= (tmp1.size() - 1);
        Eigen::Matrix2f C;
        C(0,0) = cov_mat[0];
        C(0,1) = C(1,0) = cov_mat[1];
        C(1,1) = cov_mat[1];
        Eigen::EigenSolver<Eigen::Matrix2f> es(C);
        //std::cout << es.eigenvectors() <<std::endl << std::endl;
        Eigen::Matrix2f vv = es.pseudoEigenvectors();
        maindirect[seg].x = vv(0,1);
        maindirect[seg].y = vv(1,1);
        //Eigen::Vector2f e_vector1 = es.eigenvectors().col(0);
        //Eigen::Vector2f e_vector2 = es.eigenvectors().col(1);

    }//end seg
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

std::vector<cv::Point2f> ShapeEstimator::convert2CVPoint_P(pcl::PointCloud<pcl::PointXYZL>::Ptr in)
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

std::vector<cv::Point2f> ShapeEstimator::findNearest_P(pcl::PointCloud<pcl::PointXYZL>::Ptr in)
{
    std::vector<cv::Point2f> tmp_(1800);//0.2
    std::vector<float> dis_(1800, 0);
    for(int pp = 0; pp < in->points.size(); ++pp){
        pcl::PointXYZL tp = in->points[pp];
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

std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr > ShapeEstimator::pointCloudSeparate(pcl::PointCloud<pcl::PointXYZL> labeled_pc, int label_num)
{
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr > out;
    pcl::PointCloud<pcl::PointXYZL>::Ptr aaa;
    out.push_back(aaa);
    for(int seg = 1; seg <= label_num; ++seg){
        pcl::PointCloud<pcl::PointXYZL>::Ptr tp_ptr(new pcl::PointCloud<pcl::PointXYZL>);
        tp_ptr ->height = 1;
        for(int pp = 0; pp < labeled_pc.points.size();++pp ){
            if(labeled_pc.points[pp].label == seg){
                tp_ptr->points.push_back(labeled_pc.points[pp]);
            }
        }//end pc
        tp_ptr->width = tp_ptr->points.size();
        out.push_back(tp_ptr);
    }//end seg
    return out;
}

cv::Point2f ShapeEstimator::findLimit_Z(pcl::PointCloud<pcl::PointXYZL>::Ptr in)
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
