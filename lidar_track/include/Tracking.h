//
// Created by mjj on 17-5-18.
//

#ifndef PROJECT_TRACKING_H
#define PROJECT_TRACKING_H

#include <precision_tracking/tracker.h>
#include <precision_tracking/track_manager.h>
#include <precision_tracking/sensor_specs.h>
#include <boost/make_shared.hpp>
#include <fstream>
#include <iostream>


class Tracking{
public:
    Tracking();
    ~Tracking(){};
    void pushNewFrame(const std::vector<pcl::PointCloud<pcl::PointXYZI> > in_clusters, const double& timeStamp);
    void genNewFrames(const std::vector<pcl::PointCloud<pcl::PointXYZI> > in_clusters, const double& timeStamp,
                      std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> >& in_frames);
    void association(const std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> > in_frames,
                     std::vector<precision_tracking::Tracker>& trackers,const double timeStamp,std::map<int,int>& asso);
    void track(const std::vector< boost::shared_ptr<precision_tracking::track_manager::Frame> > in_frames,std::map<int,int> asso,
               std::vector<precision_tracking::Tracker>& trackers);
    void deleteVanishTrack(std::vector<precision_tracking::Tracker>& trackers, double timeStamp);
    void findCurVelocity(const double timeStamp,double& cur_self_velocity,double& cur_self_yaw);
    void comPuteCurTracksAbsVelocity(std::vector<precision_tracking::Tracker>& trackers,
                                     const double& cur_self_velocity,const double& cur_self_yaw);

    float compute3d(Eigen::Vector3f center1,Eigen::Vector3f center2);
    //-------------读取obd自身速度
    void readOBD(std::string obdPath);
    void readICP(std::string ocpPath);

//private:
    double saveMaxTime;
    int ignorNum;
    precision_tracking::Params params;
    std::vector<precision_tracking::Tracker> trackers;
    std::vector<std::vector<double> > self_velocity;
    std::vector<std::vector<double> > self_yaw;
    bool OBDFlag,ICPFlag;



    double associaDisThre1,associaDisThre2;
    double associaSizeThre1,associaSizeThre2;
    double associaSigmaThreFactor;
    double associaNumThre1Factor,associaNumThre2Factor;

    int onlyIdForTracker;

};

#endif //PROJECT_TRACKING_H
