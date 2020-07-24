
#include <map>
#include "mappoint.h"


size_t MapPoint::last_idx=0;


MapPoint::MapPoint(const cv::Point3d &p,const cv::Scalar &color):mPoint(p),mColor(color),idx(last_idx++),is_valid(true){}

void MapPoint::AddFrame(size_t frame_idx,size_t keypoint_idx){
    mpObservations[frame_idx]=keypoint_idx;
}

Eigen::Vector3d MapPoint::Postion()const{
    return Eigen::Vector3d(mPoint.x,mPoint.y,mPoint.z);
}

void MapPoint::SetPostion(const Eigen::Vector3d &v) {
    mPoint.x=v.x();
    mPoint.y=v.y();
    mPoint.z=v.z();
}