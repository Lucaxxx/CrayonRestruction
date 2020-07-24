
#include <Eigen/Core>

#include "frame.h"


size_t Frame::next_idx=0;
cv::Ptr<cv::xfeatures2d::SIFT> Frame::SiftDetector=cv::xfeatures2d::SIFT::create();


void Frame::UndistortKeyPoints(){
    vector<cv::Point2f> keypointsun;
    for(const auto &it:mvKeyPointsUn){
        keypointsun.push_back(it.pt);
    }
    undistortPoints(keypointsun,keypointsun,K,distCoeffs,cv::Mat(),K);

    for(int i=0;i<mvKeyPointsUn.size();i++)
        mvKeyPointsUn[i].pt=keypointsun[i];
}


Frame::Frame(const std::string& filename)
:mRot(cv::Mat::eye(3,3,CV_64F)),mTrans(cv::Mat::zeros(1,3,CV_64F)),idx(next_idx++),is_init(false){

    mImage=imread(filename,cv::IMREAD_REDUCED_COLOR_2);

    SiftDetector->detectAndCompute(mImage,cv::Mat(),mvKeyPointsUn,mDescriptors);

    for(const auto &it:mvKeyPointsUn) {
        mvColors.push_back(mImage.at<cv::Vec3b>(it.pt));
    }

    //UndistortKeyPoints();

}

void Frame::AddMapPoint(const size_t keypoint_idx,const size_t mappoint_idx){
    mmMapPoints[keypoint_idx]=mappoint_idx;
}


void Frame::SetPose(const cv::Mat& R,const cv::Mat& t){
    is_init=true;
    mRot=R.clone();
    mTrans=t.clone();
}

void Frame::SetPose(const g2o::SE3Quat &SE3) {
    Eigen::Matrix4d M=SE3.to_homogeneous_matrix();
    cv::Mat R=(cv::Mat_<double>(3,3)<<M(0,0),M(0,1),M(0,2),
                                        M(1,0),M(1,1),M(1,2),
                                        M(2,0),M(2,1),M(2,2));
    cv::Mat t=(cv::Mat_<double>(3,1)<<M(0,3),M(1,3),M(2,3));
    mRot=R.clone();
    mTrans=t.clone();
}


g2o::SE3Quat Frame::Pose() {

    Eigen::Matrix3d R;
    R<<mRot.at<double>(0,0),mRot.at<double>(0,1),mRot.at<double>(0,2),
       mRot.at<double>(1,0),mRot.at<double>(1,1),mRot.at<double>(1,2),
       mRot.at<double>(2,0),mRot.at<double>(2,1),mRot.at<double>(2,2);
    Eigen::Vector3d t;
    t<<mTrans.at<double>(0,0),mTrans.at<double>(1,0),mTrans.at<double>(2,0);
    return g2o::SE3Quat(R,t);

}

cv::Mat Frame::P()const{
    return K*(cv::Mat_<double > (3,4) <<
            mRot.at<double>(0,0),mRot.at<double>(0,1),mRot.at<double>(0,2),mTrans.at<double>(0,0),
            mRot.at<double>(1,0),mRot.at<double>(1,1),mRot.at<double>(1,2),mTrans.at<double>(0,1),
            mRot.at<double>(2,0),mRot.at<double>(2,1),mRot.at<double>(2,2),mTrans.at<double>(0,2));
}

cv::Mat Frame::R()const{
    return mRot;
}

cv::Mat Frame::t()const{
    return mTrans;
}
