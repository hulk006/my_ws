//
// Created by mjj on 17-5-18.
//

#include <opencv2/core/core.hpp>
#include "Tracking.h"

Tracking::Tracking(){
    saveMaxTime = 1.5;
    ignorNum = 30;//小于20个点的frame则忽略
    associaDisThre1 = 4.;
    associaDisThre2 = 3.;
    associaSizeThre1 = 2.;
    associaSizeThre2 = 3.;
    associaSigmaThreFactor = 0.4;
    associaNumThre1Factor = 0.4;
    associaNumThre2Factor = 0.6;
    onlyIdForTracker = 0;
    OBDFlag = false;
    ICPFlag = false;
}

void Tracking::readOBD(std::string obdPath){
    std::ifstream ifile;
    ifile.open(obdPath.c_str());
    assert(ifile.is_open());

    self_velocity.clear();
    std::string str;
    while (!ifile.eof())
    {
        getline(ifile, str);
        std::istringstream infile(str);
        std::vector<double> tmp_self_velocity;
        std::string tmp_c;
        infile >> tmp_c;
        double tmp = atof(tmp_c.c_str()) / 1.e6;
        tmp_self_velocity.push_back(tmp);
        infile >> tmp_c;
        tmp_self_velocity.push_back(atof(tmp_c.c_str()) / 3.6);
        self_velocity.push_back(tmp_self_velocity);
    }
    OBDFlag = true;
}

void Tracking::readICP(std::string ocpPath){
    std::ifstream ifile;
    ifile.open(ocpPath.c_str());
    assert(ifile.is_open());

    self_yaw.clear();
    std::string str;
    while (!ifile.eof())
    {
        getline(ifile, str);
        std::istringstream infile(str);
        std::vector<double> tmp_self_yaw;
        std::string tmp_c;
        infile >> tmp_c;
        double tmp = atof(tmp_c.c_str());
        double aa = (int)tmp;
        double bb = tmp - (int)tmp;
        tmp = aa + bb * 1000;
        tmp_self_yaw.push_back(tmp);
        for (int i = 1; i < 6; ++i) {
            infile >> tmp_c;
        }

        infile>>tmp_c;
        double yaw = atof(tmp_c.c_str());
        tmp_self_yaw.push_back(yaw);
        self_yaw.push_back(tmp_self_yaw);
    }
    ICPFlag = true;
}

void Tracking::findCurVelocity(const double timeStamp,double& cur_self_velocity,double& cur_self_yaw){
    if (self_velocity.size() <= 0){
        cur_self_velocity = 0.;
        cur_self_yaw = 0.;
        return;
    }
    if (self_yaw.size() <= 0){
        cur_self_yaw = 0.;
    }else{
        cur_self_yaw = 0.;
        for (int i = 0; i < self_yaw.size() - 1; ++i) {
            double cur_stamp = self_yaw[i][0];
            double next_stamp = self_yaw[i+1][0];
            if (timeStamp > cur_stamp && timeStamp < next_stamp){
                cur_self_yaw = (timeStamp - cur_stamp) > (next_stamp - timeStamp) ? self_yaw[i + 1][1] : self_yaw[i][1];
                break;
            }
        }
    }

    cur_self_velocity = 0.;
    for (int i = 0; i < self_velocity.size() - 1; ++i) {
        double cur_stamp = self_velocity[i][0];
        double next_stamp = self_velocity[i+1][0];
        if (timeStamp > cur_stamp && timeStamp < next_stamp){
            cur_self_velocity = (timeStamp - cur_stamp) > (next_stamp - timeStamp) ? self_velocity[i + 1][1] : self_velocity[i][1];
        }
    }


}

void Tracking::pushNewFrame(const std::vector<pcl::PointCloud<pcl::PointXYZI> > in_clusters, const double& timeStamp){

    double cur_self_velocity,cur_self_yaw;
    //-------通过时间戳，寻找最接近时间戳的self_velocity和self_yaw
    findCurVelocity(timeStamp,cur_self_velocity,cur_self_yaw);
    //---------------将新的一帧点云分割转换成frames
    std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> > in_frames;
    genNewFrames(in_clusters, timeStamp, in_frames);
    //---------------数据关联
    std::map<int,int> asso; // first表示tracker，second表示frames
    association(in_frames,trackers,timeStamp,asso);//暂时先用GNN进行数据关联吧，之后再合杨浩的关联
    //---------------跟踪
    track(in_frames,asso,trackers);
    //---------------删除消失的frame
    deleteVanishTrack(trackers,timeStamp);
    comPuteCurTracksAbsVelocity(trackers,cur_self_velocity,cur_self_yaw);

}

void Tracking::comPuteCurTracksAbsVelocity(std::vector<precision_tracking::Tracker>& trackers,
                                           const double& cur_self_velocity,const double& cur_self_yaw){
    if (OBDFlag){
        if (ICPFlag){
            double cur_self_vx,cur_self_vy;
            cur_self_vx = cur_self_velocity * cos(cur_self_yaw);
            cur_self_vy = cur_self_velocity * sin(cur_self_yaw);
            for (int i = 0; i < trackers.size(); ++i) {
                if (!trackers[i].isCurFrame)continue;
                if (trackers[i].isNewTrack){
                    trackers[i].cur_y_abs_velocity = 0.;
                    trackers[i].cur_x_abs_velocity = 0.;
                    trackers[i].cur_angle_in_own_xaxi_ = 0.;
                    continue;
                }

                Eigen::Vector3f cur_track_velocity = trackers[i].get_motion_model().get_mean_velocity();
                float cur_track_vx = cur_track_velocity(0);
                float cur_track_vy = cur_track_velocity(1);

                float cur_track_transVx = cur_track_vy;//因为icp坐标与相对坐标系相反，所以需要转换
                float cur_track_transVy = cur_track_vx;

                float cur_track_rotateVx = cur_track_transVx * cos(cur_self_yaw) - cur_track_transVy * sin(cur_self_yaw);
                float cur_track_rotateVy = cur_track_transVx * sin(cur_self_yaw) + cur_track_transVy * cos(cur_self_yaw);

                float cur_track_absluteVx = cur_track_rotateVx + cur_self_vx;
                float cur_track_absluteVy = cur_track_rotateVy + cur_self_vy;

                trackers[i].cur_x_abs_velocity = cur_track_absluteVx;
                trackers[i].cur_y_abs_velocity = cur_track_absluteVy;

                float cur_abs_angle = atan2(cur_track_absluteVy , cur_track_absluteVx);
                float v_angle_in_own_xaxis_ = M_PI_2 - (cur_abs_angle - cur_self_yaw);
                v_angle_in_own_xaxis_ = v_angle_in_own_xaxis_ > M_PI ? (v_angle_in_own_xaxis_ - M_PI) : v_angle_in_own_xaxis_;
                trackers[i].cur_angle_in_own_xaxi_ = v_angle_in_own_xaxis_;
            }
        }else{
            //只有obd信息的时候，认为yaw为0
            for (int i = 0; i < trackers.size(); ++i) {
                Eigen::Vector3f cur_track_velocity = trackers[i].get_motion_model().get_mean_velocity();
                float cur_track_vx = cur_track_velocity(0);
                float cur_track_vy = cur_track_velocity(1);
                trackers[i].cur_x_abs_velocity = cur_track_vy + cur_self_velocity;
                trackers[i].cur_y_abs_velocity = cur_track_vx;

                float cur_abs_angle = atan2(trackers[i].cur_y_abs_velocity , trackers[i].cur_x_abs_velocity);
                float v_angle_in_own_xaxis_ = M_PI_2 - (cur_abs_angle - cur_self_yaw);
                v_angle_in_own_xaxis_ = v_angle_in_own_xaxis_ > M_PI ? (v_angle_in_own_xaxis_ - M_PI) : v_angle_in_own_xaxis_;
                trackers[i].cur_angle_in_own_xaxi_ = v_angle_in_own_xaxis_;
            }
        }
    }else{
        //obd和icp信息都没有的时候
        for (int i = 0; i < trackers.size(); ++i) {
            Eigen::Vector3f cur_track_velocity = trackers[i].get_motion_model().get_mean_velocity();
            float cur_track_vx = cur_track_velocity(0);
            float cur_track_vy = cur_track_velocity(1);
            trackers[i].cur_x_abs_velocity = cur_track_vy;
            trackers[i].cur_y_abs_velocity = cur_track_vx;

            float cur_abs_angle = atan2(trackers[i].cur_y_abs_velocity , trackers[i].cur_x_abs_velocity);
            float v_angle_in_own_xaxis_ = M_PI_2 - (cur_abs_angle - cur_self_yaw);
            v_angle_in_own_xaxis_ = v_angle_in_own_xaxis_ > M_PI ? (v_angle_in_own_xaxis_ - M_PI) : v_angle_in_own_xaxis_;
            trackers[i].cur_angle_in_own_xaxi_ = v_angle_in_own_xaxis_;
        }
    }


}

void Tracking::genNewFrames(const std::vector<pcl::PointCloud<pcl::PointXYZI> > in_clusters,const double& timeStamp,
                            std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> >& in_frames){
    in_frames.clear();
    for (int i = 0; i < in_clusters.size() - 1; ++i) {
        if (in_clusters[i].size() < ignorNum)continue;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        *tmp_cloud_ptr = in_clusters[i];
        boost::shared_ptr<precision_tracking::track_manager::Frame> tmp_frame(new precision_tracking::track_manager::Frame(tmp_cloud_ptr,timeStamp));
        in_frames.push_back(tmp_frame);
    }
}

void Tracking::deleteVanishTrack(std::vector<precision_tracking::Tracker>& trackers, double timeStamp){
    for (int i = trackers.size() - 1; i >= 0 ; --i) {
        double curTrackTime = trackers[i].prev_timestamp_;
        if (timeStamp - curTrackTime > saveMaxTime){
            trackers.erase(trackers.begin() + i);
        }
    }
}

bool compY(cv::Point3f pt1,cv::Point3f pt2){
    return pt1.y < pt2.y;
}
void Tracking::association(const std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> > in_frames,
                           std::vector<precision_tracking::Tracker>& trackers,const double timeStamp,std::map<int,int>& asso){
    //------------标记track和frame是否关联上
    std::vector<bool> assInFrameTag,assTrackTag;
    assInFrameTag.resize(in_frames.size());
    for (int i = 0; i < in_frames.size(); ++i) {
        assInFrameTag[i] = false;
    }
    assTrackTag.resize(trackers.size());
    for (int i = 0; i < trackers.size(); ++i) {
        assTrackTag[i] = false;
    }
    //-----------track根据时间预测位置
    std::vector<Eigen::Vector3f> propagate_positins;
    std::vector<double> time_diff;
    propagate_positins.resize(trackers.size());
    time_diff.resize(trackers.size());
    for (int i = 0; i < trackers.size(); ++i) {
        float timestamp_diff = std::max(timeStamp - trackers[i].prev_timestamp_,0.01);
        time_diff[i] = timestamp_diff;
        Eigen::Vector3f pre_position = *(trackers[i].centroid_);
        propagate_positins[i] = pre_position;// + trackers[i].get_motion_model().get_mean_velocity() * timestamp_diff;
    }

    //********************GNN asso***************
    std::vector<std::vector<cv::Point3f> > disRecord;
    disRecord.resize(in_frames.size());
    for (int j = 0; j < in_frames.size(); ++j) {
        //-------计算track在预测位置后，与frame距离，并进行排序
        disRecord[j].resize(propagate_positins.size());
        for (int i = 0; i < propagate_positins.size(); ++i) {
            disRecord[j][i].x = i;
            if (trackers[i].isCurFrame){
                disRecord[j][i].y = compute3d(propagate_positins[i], in_frames[j]->getCentroid());
            }else{
                disRecord[j][i].y = INT_MAX;
            }
            disRecord[j][i].z = time_diff[i];
        }
        std::sort(disRecord[j].begin(), disRecord[j].end(), compY);
        //--------根据距离，size的差值，size的比值以及点数的差值来进行数据关联
        for (int i = 0; i < disRecord[j].size(); ++i) {
            if (disRecord[j][i].y > associaDisThre1)break;
            if (assTrackTag[disRecord[j][i].x])continue;

            asso.insert(std::pair<int,int>(disRecord[j][i].x,j));
            assTrackTag[i] = true;
            assInFrameTag[j] = true;
            break;
        }
    }
    //*********************************/


    /************目前这种数据关联有问题，需要确认具体是什么原因
    //---------进行第一次关联，在一个限定较严的阈值里
    std::vector<std::vector<cv::Point3f> > disRecord;
    disRecord.resize(in_frames.size());
    for (int j = 0; j < in_frames.size(); ++j) {
        //-------计算track在预测位置后，与frame距离，并进行排序
        disRecord[j].resize(propagate_positins.size());
        for (int i = 0; i < propagate_positins.size(); ++i) {
            disRecord[j][i].x = i;
            disRecord[j][i].y = compute3d(propagate_positins[i],in_frames[j]->getCentroid());
            disRecord[j][i].z = time_diff[i];
        }
        std::sort(disRecord[j].begin(),disRecord[j].end(),compY);
        //--------根据距离，size的差值，size的比值以及点数的差值来进行数据关联
        for (int i = 0; i < disRecord[j].size(); ++i) {
            if (disRecord[j][i].y > associaDisThre1)break;
            if (assTrackTag[disRecord[j][i].x])continue;

            Eigen::MatrixXf boudingTrack = *(trackers[i].bounding_box_);
            Eigen::MatrixXf boudingFrame = in_frames[j]->getBoundingBox();
            float trackX = boudingTrack(0,1) - boudingTrack(0,0);
            float trackY = boudingTrack(1,1) - boudingTrack(1,0);
            float frameX = boudingFrame(0,1) - boudingFrame(0,0);
            float frameY = boudingFrame(1,1) - boudingFrame(1,0);
            float deltaSize = fabs(trackX + trackY -(frameX + frameY));
            float sigmaSize = (trackX / trackY) / (frameX / frameY);
            sigmaSize = sigmaSize < 1. ? sigmaSize : 1. / sigmaSize;
            int deltaNum = abs(trackers[i].previousModel_->size() - in_frames[j]->cloud_->size());
            int threNum = trackers[i].previousModel_->size() > in_frames[j]->cloud_->size() ?
                          associaNumThre1Factor * trackers[i].previousModel_->size() : associaNumThre1Factor * in_frames[j]->cloud_->size();

            std::cout<<trackers[i].previousModel_->size()<<" "<<in_frames[j]->cloud_->size()<<std::endl;
            if (deltaNum < threNum && deltaSize < associaSizeThre1 && sigmaSize > associaSigmaThreFactor){
                asso.insert(std::pair<int,int>(disRecord[j][i].x,j));
                assTrackTag[i] = true;
                assInFrameTag[j] = true;
                break;
            }
        }
    }

    //------------进行第二次数据关联，此次对于未关联上的目标给出一个更轻松的阈值限定

    for (int j = 0; j < in_frames.size(); ++j) {
        if (assInFrameTag[j])continue;
        //--------根据距离，size的差值，size的比值以及点数的差值来进行数据关联
        for (int i = 0; i < disRecord[j].size(); ++i) {
            if (disRecord[j][i].y > associaDisThre2)break;
            if (assTrackTag[disRecord[j][i].x])continue;

            Eigen::MatrixXf boudingTrack = *(trackers[i].bounding_box_);
            Eigen::MatrixXf boudingFrame = in_frames[j]->getBoundingBox();

            float trackX = boudingTrack(1,0) - boudingTrack(0,0);
            float trackY = boudingTrack(1,1) - boudingTrack(1,0);
            float frameX = boudingFrame(1,0) - boudingFrame(0,0);
            float frameY = boudingFrame(1,1) - boudingFrame(1,0);
            float deltaSize = fabs(trackX + trackY -(frameX + frameY));
            float sigmaSize = (trackX / trackY) / (frameX / frameY);
            sigmaSize < 1. ? sigmaSize : 1. / sigmaSize;
            int deltaNum = abs(trackers[i].previousModel_->size() - in_frames[j]->cloud_->size());
            int threNum = trackers[i].previousModel_->size() > in_frames[j]->cloud_->size() ?
                          associaNumThre2Factor * trackers[i].previousModel_->size() : associaNumThre2Factor * in_frames[j]->cloud_->size();

            if (deltaNum < threNum && deltaSize < associaSizeThre2 && sigmaSize < associaSigmaThreFactor){
                asso.insert(std::pair<int,int>(disRecord[j][i].x,j));
                assTrackTag[i] = true;
                assInFrameTag[j] = true;
                break;
            }
        }
    }
    //*/
}

float Tracking::compute3d(Eigen::Vector3f center1,Eigen::Vector3f center2){
    return sqrtf(powf(center1.coeff(0) - center2.coeff(0),2)
                 + powf(center1.coeff(1) - center2.coeff(1),2) + powf(center1.coeff(2) - center2.coeff(2),2));
}

void Tracking::track(const std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> > in_frames,std::map<int,int> asso,
                     std::vector<precision_tracking::Tracker>& trackers){

    std::vector<bool> assoFrameTag;
    assoFrameTag.resize(in_frames.size());
    for (int i = 0; i < in_frames.size(); ++i) {
        assoFrameTag[i] = false;
    }

    for (int i = 0; i < trackers.size(); ++i) {
        std::map<int,int>::iterator iter;
        iter = asso.find(i);
        if (iter == asso.end())
        {
            trackers[i].isCurFrame = false;
            continue;
        }
        trackers[i].isCurFrame = true;
        trackers[i].isNewTrack = false;
        int frameIdx = (*iter).second;
        assoFrameTag[frameIdx] = true;
        const boost::shared_ptr<precision_tracking::track_manager::Frame> frame = in_frames[frameIdx];
        // Get the sensor resolution.
        double sensor_horizontal_resolution;
        double sensor_vertical_resolution;
        precision_tracking::getSensorResolution(
                frame->getCentroid(), &sensor_horizontal_resolution,
                &sensor_vertical_resolution);
        // Track object.
        Eigen::Vector3f estimated_velocity;
        trackers[i].addPoints(frame->cloud_,frame->timestamp_,
                              sensor_horizontal_resolution,
                              sensor_vertical_resolution,
                              &estimated_velocity);
    }

    for (int i = 0; i < assoFrameTag.size(); ++i) {
        if (assoFrameTag[i])continue;
        precision_tracking::Tracker tracker(&params);
        tracker.setPrecisionTracker(
                boost::make_shared<precision_tracking::PrecisionTracker>(&params));
        tracker.uiniqueID = onlyIdForTracker;
        onlyIdForTracker++;
        tracker.isCurFrame = true;
        tracker.isNewTrack = true;
        const boost::shared_ptr<precision_tracking::track_manager::Frame> frame = in_frames[i];

        // Get the sensor resolution.
        double sensor_horizontal_resolution;
        double sensor_vertical_resolution;
        precision_tracking::getSensorResolution(
                frame->getCentroid(), &sensor_horizontal_resolution,
                &sensor_vertical_resolution);
        // Track object.
        Eigen::Vector3f estimated_velocity;
        tracker.addPoints(frame->cloud_,frame->timestamp_,
                          sensor_horizontal_resolution,
                          sensor_vertical_resolution,
                          &estimated_velocity);
        trackers.push_back(tracker);
    }
}