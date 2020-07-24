#ifndef CRAYONRESTRUCTION_TRACKER_H
#define CRAYONRESTRUCTION_TRACKER_H

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "frame.h"

using std::map;
using std::string;
using std::vector;
using std::cout;
using std::endl;


class Tracker{

public:

    Tracker(const string& camerayaml,const string& pictureyaml);

    bool Tracking();

    void Triangulate(Frame& frame1,Frame& frame2,const vector<cv::Point2d>& vpoint1,const vector<cv::Point2d>& vpoint2,const vector<cv::DMatch>& matchers);

    bool Match2D2D(Frame& frame1,Frame& frame2);

    bool Match3D2D(Frame& frame1,Frame& frame2);


    void RejectwithF(vector<cv::Point2d>& vpoint1,vector<cv::Point2d>& vpoint2,vector<cv::DMatch>& matchers,const cv::Mat& F);

    void AddObervation(Frame& frame1,Frame& frame2,const vector<cv::Point3d>& vpoint3d,const vector<cv::Point2d>& vpoint2d,const vector<cv::DMatch>& matchers,double threshold=2.0);

    void RemoveOutliers();

    vector<Frame> mvFrames;

    vector<MapPoint> mvMapPoints;

};

#endif