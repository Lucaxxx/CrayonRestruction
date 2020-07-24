#ifndef CRAYONRESTRUCTION_CALIBRATION_H
#define CRAYONRESTRUCTION_CALIBRATION_H


#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

using std::string;
using std::map;
using std::vector;
using std::cout;
using std::endl;

class Calibration{
public:

    Calibration(const string& filename);


    void Calibrate();

    void print()const;

    void write(const string& filename)const;

    vector<cv::Mat> mvImages;

    cv::Size mSize;

    vector<vector<cv::Point2f>> mvCorners;
    vector<vector<cv::Point3f>> mvPoints;

    cv::Mat CameraMatrix;
    cv::Mat DistCoeffs;

    cv::Mat rvecs,tvecs;

};

#endif