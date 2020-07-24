#ifndef CRAYONRESTRUCTION_OPTIMIZER_H
#define CRAYONRESTRUCTION_OPTIMIZER_H


#include "frame.h"
#include "mappoint.h"

#include <unistd.h>

using std::string;
using std::map;
using std::vector;
using std::cout;
using std::endl;

class Optimizer{
public:
    void BundleAdjustmentSE3(vector<Frame>& frames,vector<MapPoint>& mappoints,cv::Mat& K ,size_t iterations=100);

    void BundleAdjustmentSim3(vector<Frame>& frames,vector<MapPoint>& mappoints,cv::Mat& K ,size_t iterations=100);

};

#endif //CRAYONRESTRUCTION_OPTIMIZER_H
