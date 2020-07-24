

#include "calibration.h"


Calibration::Calibration(const string& filename){
    cv::FileStorage fs(filename,cv::FileStorage::READ);

    fs["Cols"]>>mSize.height;
    fs["Rows"]>>mSize.width;

    cv::FileNode fn=fs["calibration_images"];
    for(auto it:fn){
        cv::Mat image=imread((std::string)it,cv::IMREAD_REDUCED_COLOR_2);
        mvImages.push_back(image);
    }
}

void Calibration::Calibrate(void){

    vector<cv::Point3f> vp;
    for(int i=0;i<mSize.height;i++)
        for(int j=0;j<mSize.width;j++) {
            cv::Point3d p(i * 1.0, j * 1.0, 0.0);
            vp.push_back(p);
        }


    for(int i=0;i<mvImages.size();i++){
        vector<cv::Point2f> corners;
        bool PatternWasFound=findChessboardCorners(mvImages[i],mSize,corners,cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
        mvCorners.push_back(corners);
        mvPoints.push_back(vp);
        //drawChessboardCorners(mvImages[i],mSize,corners,PatternWasFound);
        //imshow("debug",mvImages[i]);
        //waitKey(0);
    }

    calibrateCamera(mvPoints,mvCorners,mvImages[0].size(),CameraMatrix,DistCoeffs,rvecs,tvecs,cv::CALIB_FIX_PRINCIPAL_POINT);
}

void Calibration::print()const{
    cout<<"Camera intrinsic:"<<endl<<CameraMatrix<<endl;
    cout<<"Distort Coefficients"<<endl<<DistCoeffs<<endl;
}

void Calibration::write(const string& filename)const{
    cv::FileStorage fs(filename,cv::FileStorage::WRITE);
    fs<<"projection_parameters"<<CameraMatrix;
    fs<<"distortion_parameters"<<DistCoeffs;
}
