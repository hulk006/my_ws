#include"track_label.h"

bool Segment::deserialize(std::istream &istrm)
{
    std::string line;
    std::getline(istrm, line);
    if(line.compare("seg") != 0)
    {
        std::cout << "Expected 'seg', got " << line << std::endl;
        return false;
    }

    std::getline(istrm, line);
    if(line.compare("segPointsNum") != 0)
    {
        std::cout << "Expected 'segPointsNum', got " << line << std::endl;
        return false;
    }
    int pointnum = 0;
    istrm >> pointnum;
    std::getline(istrm, line);

    cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_->header.frame_id = "velodyne";
    cloud_->height = 1;

    for(int pp = 0; pp < pointnum;++pp )
    {
        pcl::PointXYZI p;
        istrm.read((char*)&p.x,sizeof(double));
        istrm.read((char*)&p.y,sizeof(double));
        istrm.read((char*)&p.z,sizeof(double));
        istrm.read((char*)&p.intensity,sizeof(double));
        std::getline(istrm, line);
        cloud_->points.push_back(p);
    }
    cloud_->width = cloud_->points.size();
    return true;

}

void Track::serialize(std::string &filename)
{
    std::ofstream trackfile;
    trackfile.open(filename.c_str(), std::ios::app);
    trackfile<<"Track"<<std::endl;
    trackfile<<"label"<<std::endl;
    trackfile<<label_<<std::endl;
    trackfile<<"num_frames"<<std::endl;
    trackfile<<segments_.size()<<std::endl;
    for(int seg_num = 0; seg_num < segments_.size();seg_num++)
    {
        trackfile<<"seg"<<std::endl;
        trackfile<<"segPointsNum"<<std::endl;
        trackfile<<segments_[seg_num] -> cloud_->points.size()<<std::endl;
        for(int pp = 0; pp < segments_[seg_num] -> cloud_->points.size();++pp)
        {
            pcl::PointXYZI point = segments_[seg_num] -> cloud_->points[pp];
            trackfile.write((char*)&point.x, sizeof(double));
            trackfile.write((char*)&point.y, sizeof(double));
            trackfile.write((char*)&point.z, sizeof(double));
            trackfile.write((char*)&point.intensity, sizeof(double));
            trackfile<<std::endl;
        }
    }
    trackfile.close();

}

bool Track::deserialize(std::istream& istrm)
{
    if(istrm.eof())
        return false;

    long begin = istrm.tellg();
    segments_.clear();

    std::string line;

    std::getline(istrm,line);
    if(line.compare("Track") != 0)
    {
        istrm.seekg(begin);
        std::cout << "Expected 'Track', got " << line << std::endl;
        return false;
    }

    std::getline(istrm,line);
    if(line.compare("label") != 0)
    {
        istrm.seekg(begin);
        std::cout << "Expected 'label', got " << line << std::endl;
        return false;
    }

    std::getline(istrm,line);
    label_ = line;

    std::getline(istrm,line);
    if(line.compare("num_frames") != 0)
    {
        istrm.seekg(begin);
        std::cout << "Expected 'num_frames', got " << line << std::endl;
        return false;
    }

    int num_segments = 0 ;
    istrm>>num_segments;
    std::getline(istrm,line);

    segments_.resize(num_segments);
    for(int i = 0; i < num_segments; ++i)
    {
        assert(!segments_[i]);
        segments_[i] = boost::shared_ptr<Segment>(new Segment());
        segments_[i] -> deserialize(istrm);

    }

    return true;
}

TrackManager::TrackManager(const std::string &filename)
{
    std::ifstream file(filename.c_str(),std::ios::in);
    assert(file.is_open());
    bool success = deserialize(file);
    assert(success);
    file.close();
}

bool TrackManager::deserialize(std::istream &istrm)
{
    tracks_.clear();

    while(true)
    {
        boost::shared_ptr<Track> tr(new Track());
        if(tr->deserialize(istrm))
            tracks_.push_back(tr);
        else
            break;
    }
    return true;

}
