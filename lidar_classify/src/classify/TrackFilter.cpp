#include "TrackFilter.h"

TrackFilter::TrackFilter(vector<float> vprob)
{
    prob = 0;
    computeFilterProb(vprob);
}

void TrackFilter::computeFilterProb(vector<float> vprob)
{
    if(vprob.size() == 0)
    {
        cout<<"input prob vector size is 0!"<<endl;
        return;
    }
    float probSum = 0;
    for(int i = 0; i< vprob.size(); i++)
    {
        probSum += vprob[i];
    }
    prob = probSum/vprob.size();
}
