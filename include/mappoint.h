#ifndef CRAYONRESTRUCTION_MAPPOINT_H
#define CRAYONRESTRUCTION_MAPPOINT_H

#include <vector>
#include <map>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

using std::string;
using std::map;
using std::vector;
using std::cout;
using std::endl;

class Frame;

class MapPoint{

public:

    explicit MapPoint(const cv::Point3d &p,const cv::Scalar& color);

    void AddFrame(size_t frame_idx,size_t keypoint_idx);

    void SetPostion(const Eigen::Vector3d& v);

    Eigen::Vector3d Postion()const;

    cv::Point3d mPoint;

    cv::Scalar mColor;

    //key: observable frame,value: keypoint idx
    map<size_t,size_t> mpObservations;

    bool is_valid;

    size_t idx;
    static  size_t last_idx;

};

#endif