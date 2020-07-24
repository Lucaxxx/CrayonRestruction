#ifndef CRAYONRESTRUCTION_FRAME_H
#define CRAYONRESTRUCTION_FRAME_H


#include <string>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "g2o/types/slam3d/se3quat.h"

#include "mappoint.h"


using std::string;
using std::map;
using std::vector;
using std::cout;
using std::endl;


class Frame{

public:
    explicit Frame(const string& filename);

    void UndistortKeyPoints();

    //Projection Matrix = K*[R|t]
    cv::Mat P()const;

    void SetPose(const cv::Mat& R,const cv::Mat& t);
    void SetPose(const g2o::SE3Quat& SE3);
    g2o::SE3Quat Pose();

    void AddMapPoint(const size_t keypoint_idx,const size_t mappoint_idx);

    cv::Mat R()const ;
    cv::Mat t()const ;

    cv::Mat mImage;

    vector<cv::KeyPoint> mvKeyPointsUn;
    cv::Mat mDescriptors;

    static cv::Ptr<cv::xfeatures2d::SIFT> SiftDetector;

    //key keypoint idx,value mappoint idx
    map<size_t,size_t> mmMapPoints;


    //color(blue,green,red)
    vector<cv::Scalar> mvColors;

    //R 3*3
    cv::Mat mRot;
    //t 3*1
    cv::Mat mTrans;

    static cv::Mat K;
    static cv::Mat distCoeffs;

    static size_t next_idx;
    size_t idx;

    bool is_init;
};


#endif