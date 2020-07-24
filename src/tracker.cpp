#include <opencv2/sfm/fundamental.hpp>
#include <cmath>

#include "tracker.h"
#include "optimizer.h"

cv::Mat Frame::K=cv::Mat();
cv::Mat Frame::distCoeffs=cv::Mat();

Tracker::Tracker(const string& camerayaml,const string& pictureyaml){


    cv::FileStorage fs_c(camerayaml, cv::FileStorage::READ);
    fs_c["projection_parameters"] >> Frame::K;
    fs_c["distortion_parameters"] >> Frame::distCoeffs;

    cv::FileStorage fs_p(pictureyaml, cv::FileStorage::READ);
    cv::FileNode fn = fs_p["sfm_images"];
    vector<string> vFilenames;
    for (auto it:fn) {
        Frame frame(static_cast<string>(it));
        mvFrames.push_back(frame);
    }

}


void Tracker::Triangulate(Frame& frame1,Frame& frame2,const vector<cv::Point2d>& vpoint1,const vector<cv::Point2d>& vpoint2,const vector<cv::DMatch>& MatchPoints){

    cv::Mat P1 = frame1.P();
    cv::Mat P2 = frame2.P();

    cv::Mat HomoPoints;
    //Matrix HomoPoints(4*n):HomoCoordinate of 3d mappoints [p1 p2 p3 ... pn]
    triangulatePoints(P1,P2,vpoint1,vpoint2,HomoPoints);

    //homocoordinate of 2d point in frame 1 and frame 2
    cv::Mat p1_homo=P1*HomoPoints;
    cv::Mat p2_homo=P2*HomoPoints;

    int sgn_w1,sgn_w2,sgn_t;
    double error1,error2;

    int cnt=0;

    for(int i=0;i<HomoPoints.cols;i++){

        //init the point if it's in front of the camera
        sgn_w1=p1_homo.at<double>(2,i)>0?1:-1;
        sgn_w2=p2_homo.at<double>(2,i)>0?1:-1;

        //reprojection error
        error1=p1_homo.at<double>(0,i)/p1_homo.at<double>(2,i)+p1_homo.at<double>(1,i)/p1_homo.at<double>(2,i)-vpoint1[i].x-vpoint1[i].y;
        error2=p2_homo.at<double>(0,i)/p2_homo.at<double>(2,i)+p2_homo.at<double>(1,i)/p2_homo.at<double>(2,i)-vpoint2[i].x-vpoint2[i].y;

        //limit the reprejection error
        if(fabs(error1)<1&&fabs(error2)<1) {
            //calculate the sign of depth
            //make sure the point is in front of each camera
            sgn_t = HomoPoints.at<double>(3, i) > 0 ? 1 : -1;
            if (sgn_w1 * sgn_t > 0 && sgn_w2 * sgn_t > 0) {
                //transform into EuclideanCoordinate
                cv::Point3d p(HomoPoints.at<double>(0, i), HomoPoints.at<double>(1, i), HomoPoints.at<double>(2, i));
                p /= HomoPoints.at<double>(3, i);
                cv::Scalar color =
                        (frame1.mvColors[MatchPoints[i].queryIdx] + frame2.mvColors[MatchPoints[i].trainIdx]) / 2;
                MapPoint mp(p, color);
                //add index to mappoint
                mp.AddFrame(frame1.idx, MatchPoints[i].queryIdx);
                mp.AddFrame(frame2.idx, MatchPoints[i].trainIdx);
                mvMapPoints.push_back(mp);
                //add index to frames
                frame1.AddMapPoint(MatchPoints[i].queryIdx, mp.idx);
                frame2.AddMapPoint(MatchPoints[i].trainIdx, mp.idx);
                cnt++;
            }
        }
    }
    cout<<cnt<<" mappoints triangulated"<<endl;
}


//recover relative pose of two frames
bool Tracker::Match2D2D(Frame& frame1,Frame& frame2){

    cout<<"start initialization"<<endl;

    //step1:match the keypoints
    cv::FlannBasedMatcher FBmatcher;
    vector<cv::DMatch> matchers;
    FBmatcher.match(frame1.mDescriptors,frame2.mDescriptors,matchers);


    if(matchers.size()<200){
        cout<<"initialization fail"<<endl<<"too less match points"<<endl;
        return false;
    }
    cout<<"intialization success"<<endl;
    cout<<"track total match points : "<<matchers.size()<<endl;
    //step2:remove the outliers

    /*
    //find the minimum distance
    auto it_min=min_element(MatchPoints.begin(),MatchPoints.end(),
              [](const DMatch m1, const DMatch m2){
                  return m1.distance<m2.distance;
              });
    */
    double average;
    for(const auto &it:matchers)
        average+=it.distance/matchers.size();

    //remove the match if distance is larger than threshold
    double ThresDistance=average;
    vector<cv::Point2d> vpQuery,vpTrain;
    for(auto it=matchers.begin();it!=matchers.end();) {

        if (it->distance > ThresDistance){
            it = matchers.erase(it);
        }
        else {
            vpQuery.push_back(frame1.mvKeyPointsUn[it->queryIdx].pt);
            vpTrain.push_back(frame2.mvKeyPointsUn[it->trainIdx].pt);
            it++;
        }
    }

    cout<<"matchers after rough rejection : "<<matchers.size()<<endl;
    //step3:reover essential matrix using RANSAC method
    cv::Mat E,R,t;
    vector<uchar> status;
    E=findEssentialMat(vpQuery,vpTrain,Frame::K,cv::RANSAC,0.99,1,status);

    auto it_query=vpQuery.begin();
    auto it_train=vpTrain.begin();
    auto it_match=matchers.begin();

    //remove outliers
    for(auto it:status){
        if(!it) {
            it_query=vpQuery.erase(it_query);
            it_train=vpTrain.erase(it_train);
            it_match=matchers.erase(it_match);
        }
        else{
            it_query++;
            it_train++;
            it_match++;
        }
    }

    cout<<"matchers after checking E : "<<matchers.size()<<endl;

    /*
    cv::Mat res;
    cv::drawMatches(frame1.mImage,frame1.mvKeyPointsUn,frame2.mImage,frame2.mvKeyPointsUn,matchers,res);
    cv::imshow("2d",res);
    cv::waitKey(0);
     */


    //step4:recover R,t from E
    cv::recoverPose(E,vpQuery,vpTrain,Frame::K,R,t);
    cout<<"frame 1 pose : "<<endl<<"R : "<<R<<endl<<"t : "<<t<<endl;

    //fix frame1 as world frame
    frame1.SetPose(cv::Mat::eye(3,3,CV_64F),cv::Mat::zeros(3,1,CV_64F));
    //set frame2
    frame2.SetPose(R,t);

    Triangulate(frame1,frame2,vpQuery,vpTrain,matchers);
    cout<<"-----------------------------------------------------------"<<endl;
    return true;

}

void Tracker::AddObervation(Frame& frame1,Frame& frame2,const vector<cv::Point3d>& vpoint3d,const vector<cv::Point2d>& vpoint2d,const vector<cv::DMatch>& matchers,double threshold){
    for(int i=0;i<vpoint3d.size();i++){
        //transform into HomoCoordinate
        cv::Mat p_homo=(cv::Mat_<double>(4,1)<<vpoint3d[i].x,vpoint3d[i].y,vpoint3d[i].z,1);
        //reproject into CameraCoordinate
        cv::Mat p_proj=frame2.P()*p_homo;
        //back to EuclideanCoordinate
        p_proj/=p_proj.at<double>(2,0);
        //reprojection error
        double error=fabs(p_proj.at<double>(0,0)+p_proj.at<double>(1,0)-vpoint2d[i].x-vpoint2d[i].y);
        if(error<threshold){
            frame2.AddMapPoint(matchers[i].trainIdx,frame1.mmMapPoints[matchers[i].queryIdx]);
            mvMapPoints[frame1.mmMapPoints[matchers[i].queryIdx]].AddFrame(frame2.idx,matchers[i].trainIdx);
        }
    }
}




bool Tracker::Match3D2D(Frame& frame1,Frame& frame2) {

    cout<<"start tracking frame "<<frame1.idx<<" frame "<<frame2.idx<<endl;

    //step1:match the keypoints
    cv::FlannBasedMatcher FBmatcher;
    vector<cv::DMatch> matchers;
    FBmatcher.match(frame1.mDescriptors, frame2.mDescriptors, matchers);


    double average;
    for(const auto &it:matchers)
        average+=it.distance/matchers.size();
    //cout<<average<<endl;

    //remove the match if distance is larger than threshold
    double ThresDistance=average;
    vector<cv::Point2d> vpQuery,vpTrain;
    for(auto it=matchers.begin();it!=matchers.end();) {

        if (it->distance > ThresDistance){
            it = matchers.erase(it);
        }
        else {
            vpQuery.push_back(frame1.mvKeyPointsUn[it->queryIdx].pt);
            vpTrain.push_back(frame2.mvKeyPointsUn[it->trainIdx].pt);
            it++;
        }
    }


    //step3:reover essential matrix using RANSAC method
    cv::Mat E;
    vector<uchar> status;
    E=cv::findEssentialMat(vpQuery,vpTrain,Frame::K,cv::RANSAC,0.99,1,status);

    auto it_query=vpQuery.begin();
    auto it_train=vpTrain.begin();
    auto it_match=matchers.begin();

    //remove outliers
    for(auto it:status){
        if(!it) {
            it_query=vpQuery.erase(it_query);
            it_train=vpTrain.erase(it_train);
            it_match=matchers.erase(it_match);
        }
        else{
            it_query++;
            it_train++;
            it_match++;
        }
    }

    /*
    cv::Mat res;
    cv::drawMatches(frame1.mImGray,frame1.mvKeyPointsUn,frame2.mImGray,frame2.mvKeyPointsUn,matchers,res);
    cv::imshow("matchers",res);
    cv::waitKey();
     */


    //points matched with mappoints
    vector<cv::Point3d> vpQuery32;
    vector<cv::Point2d> vpTrain32;
    vector<cv::DMatch> matchers3d;

    //points to be triangulated
    vector<cv::Point2d> vpQuery22;
    vector<cv::Point2d> vpTrain22;
    vector<cv::DMatch> matchers2d;


    for (auto it:matchers) {
        if (frame1.mmMapPoints.count(it.queryIdx)&&mvMapPoints[frame1.mmMapPoints[it.queryIdx]].is_valid) {
            size_t mpidx = frame1.mmMapPoints[it.queryIdx];
            vpQuery32.push_back(mvMapPoints[mpidx].mPoint);
            vpTrain32.push_back(frame2.mvKeyPointsUn[it.trainIdx].pt);
            matchers3d.push_back(it);
        } else {
            vpQuery22.push_back(frame1.mvKeyPointsUn[it.queryIdx].pt);
            vpTrain22.push_back(frame2.mvKeyPointsUn[it.trainIdx].pt);
            matchers2d.push_back(it);
        }
    }



/*

    cv::drawMatches(frame1.mImage,frame1.mvKeyPointsUn,frame2.mImage,frame2.mvKeyPointsUn,matchers2d,res);
    cv::imshow("matchers2d",res);
    cv::waitKey();

     */


    cout<<"matched 3d points "<<matchers3d.size()<<endl;
    cout<<"matched 2d points "<<matchers2d.size()<<" left"<<endl;

    cv::Mat R, vR, t;


    cv::Rodrigues(frame1.R(), vR);
    t=frame1.t().clone();

    //vR=cv::Mat::zeros(3,1,CV_64F);
    //t=cv::Mat::zeros(3,1,CV_64F);

    if(vpQuery32.size()>=4)
        cv::solvePnP(vpQuery32, vpTrain32, Frame::K, cv::Mat(), vR, t,true);
    else{
        cv::Mat res;
        cv::drawMatches(frame1.mImage,frame1.mvKeyPointsUn,frame2.mImage,frame2.mvKeyPointsUn,matchers3d,res);
        cv::imshow("matchers3d",res);
        cv::waitKey();
        cv::drawMatches(frame1.mImage,frame1.mvKeyPointsUn,frame2.mImage,frame2.mvKeyPointsUn,matchers2d,res);
        cv::imshow("matchers2d",res);
        cv::waitKey();
        cout<<"too few matched 3d points initialize pose fail!"<<endl;
        return false;
    }

    cv::Rodrigues(vR, R);

    cout<<"recover frame "<<frame2.idx<<" pose :"<<endl<<R<<endl<<t<<endl;

    frame2.SetPose(R, t);


    AddObervation(frame1, frame2, vpQuery32, vpTrain32, matchers3d);

    Triangulate(frame1, frame2, vpQuery22, vpTrain22, matchers2d);

    cout<<"-----------------------------------------------------------"<<endl;
    return true;
}

void Tracker::RemoveOutliers(){
    for(auto mp=mvMapPoints.begin();mp!=mvMapPoints.end();mp++){
        for(auto &it:mp->mpObservations) {
            size_t frame_id = it.first;
            size_t kp_id = it.second;
            cv::Point2d p = mvFrames[frame_id].mvKeyPointsUn[kp_id].pt;
            cv::Mat p3_homo = (cv::Mat_<double>(4, 1) << mp->mPoint.x, mp->mPoint.y, mp->mPoint.z, 1);
            cv::Mat P = mvFrames[frame_id].P();
            cv::Mat p2_homo = P * p3_homo;
            double error = fabs(p2_homo.at<double>(0, 0) / p2_homo.at<double>(2, 0) +
                                p2_homo.at<double>(1, 0) / p2_homo.at<double>(2, 0) - p.x - p.y);
            if (error > 0.4) {
                mp->is_valid=false;
                break;
            }
        }
    }
}



bool Tracker::Tracking(){

    if(mvFrames.size()<2){
        cout<<"too few input images"<<endl;
        return false;
    }

    Optimizer optimizer;

    for(int i=0;i<mvFrames.size()-1;i++){
        if(i==0){
            if(!Match2D2D(mvFrames[i],mvFrames[i+1])){
                cout<<"error !"<<endl;
                break;
            }
            optimizer.BundleAdjustmentSE3(mvFrames,mvMapPoints,Frame::K,100);
            RemoveOutliers();
            cout<<"total 3d points : "<<mvMapPoints.size()<<endl;
        }
        else{
            if(!Match3D2D(mvFrames[i],mvFrames[i+1])) {
                cout << "error !" << endl;
                break;
            }
            optimizer.BundleAdjustmentSE3(mvFrames,mvMapPoints,Frame::K,100);
            RemoveOutliers();
            cout<<"total 3d points : "<<mvMapPoints.size()<<endl;
        }
    }

}