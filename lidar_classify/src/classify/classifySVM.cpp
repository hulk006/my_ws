#include "classifySVM.h"
using namespace Robosense;
ClassifierSVM::ClassifierSVM(const string &modelfile, const string &rangefile)
{
//    std::cout<<"modelfile: "<<modelfile<<std::endl;
    model_ = svm_load_model(modelfile.c_str());
    assert(model_);
    loadRange(rangefile,feature_range_);
    selectfeature_.push_back(-1);
}
ClassifierSVM::ClassifierSVM()
{
    model_ = NULL;
    selectfeature_.push_back(-1);
}

void ClassifierSVM::setModelRange(const std::string& modelfile,  const std::string& rangefile)
{
    model_ = svm_load_model(modelfile.c_str());
    loadRange(rangefile,feature_range_);
}

void ClassifierSVM::loadRange(const std::string& rangefilename, std::vector<Feature_Range>& range)
{
    range.resize(1000);
    for(int i = 0; i < range.size();++i){
        range[i].max = 0;
        range[i].min = 0;
    }
    std::ifstream file(rangefilename.c_str(),std::ios::in);
    assert(file);
    std::string line;
    std::getline(file,line);
    file >> MIN_R_;
    //std::getchar()
    file>> MAX_R_;
    std::getline(file,line);
    while(!file.eof()){
        int index;
        file >> index;
        if((index > 0) && (index < 1000)){
            file>>range[index - 1].min;
            file>>range[index - 1].max;
            std::getline(file,line);
        }
    }
    return;
}

int ClassifierSVM::objectClassify(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, vector<float> &prob)
{
    FeatureManager fm(cloud);
    std::vector<double> feature;
    fm.chooseFeature(selectfeature_,feature);
    return (int)( objectClassify(feature,prob) );
}


int ClassifierSVM::objectClassify(std::vector<double> &feature, vector<float> &prob)
{
    int classnum = model_->nr_class;
    prob.resize(classnum,0);
    rerangeFeature(feature);
    struct svm_node* class_node = new svm_node[feature.size()+1];
    for(int i = 0; i < (feature.size());++i){
        class_node[i].index = i+1;
        class_node[i].value = feature[i];
    }
//    std::cout<<feature.size()<<std::endl;
    class_node[feature.size()].index = -1;
    class_node[feature.size()].value = 0;
    double *prob_ptr = new double[classnum];
    int predictLabel =(int)svm_predict_probability(model_,class_node,prob_ptr);
    for(int i = 0; i<classnum; i++)
        prob[i] = (float)prob_ptr[i];
    delete [] class_node;
    delete prob_ptr;
    return predictLabel;
}

void ClassifierSVM::rerangeFeature(std::vector<double> &feature)
{
    for(int i = 0; i < feature.size();++i){
        double max_t = feature_range_[i].max;
        double min_t = feature_range_[i].min;
        if(max_t != min_t){
            if(feature[i] == min_t)
                feature[i] = MIN_R_;
            else if(feature[i] == max_t)
                feature[i] = MAX_R_;
            else
                feature[i] = MIN_R_ + (MAX_R_ - MIN_R_)*(feature[i] - feature_range_[i].min)/(feature_range_[i].max - feature_range_[i].min);
        }
    }
}

void ClassifierSVM::setSelectfeature(std::vector<int> sf)
{
    selectfeature_ = sf;
}

MultiClassifierSVM::MultiClassifierSVM()
{
    selectfeature_.push_back(-1);
    predictLabel_ = 0;
    predictProb_ = 0.0;
    mcf_.clear();
}

MultiClassifierSVM::MultiClassifierSVM(std::vector<string> modelRangeFile)
{
    selectfeature_.push_back(-1);
    predictLabel_ = 0;
    predictProb_ = 0.0;
    setModelRange(modelRangeFile);
}


bool MultiClassifierSVM::setModelRange(std::vector<std::string> modelRangeFile)
{
    if(modelRangeFile.size()%2 != 0)
        return false;
    int classfier_num = modelRangeFile.size()/2;
    for(int i = 0; i<classfier_num; i++)
    {
        ClassifierSVM cf(modelRangeFile[2*i],modelRangeFile[2*i+1]);
        mcf_.push_back(cf);
    }
    return true;

}

std::vector<float> MultiClassifierSVM::MultiClassify(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    std::vector<float> prob(mcf_.size(),0);
    FeatureManager fm(cloud);
    std::vector<double> feature;
    fm.chooseFeature(selectfeature_,feature);
    for(int i = 0; i<mcf_.size();i++)
    {
        assert(mcf_[i].model_->nr_class > 2);
        vector<float> tmp;
        mcf_[i].objectClassify(feature,tmp);
        prob[i] = tmp[0];
    }
    int maxclass = -1;
    int maxprob = -1;
    for(int i = 0; i<mcf_.size();i++)
    {
        if(prob[i] > maxprob)
        {
            maxprob = prob[i];
            maxclass = i;
        }
    }
    predictProb_ = maxprob;
    predictLabel_ = maxclass + 1;
    return prob;
}





