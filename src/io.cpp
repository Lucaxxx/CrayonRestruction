#include "io.h"


void io::write(const string& filename,const vector<MapPoint>& mappoints){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(const auto &it:mappoints){
        pcl::PointXYZRGB p(it.mColor[2],it.mColor[1],it.mColor[0]);
        p.x=it.mPoint.x;
        p.y=it.mPoint.y;
        p.z=it.mPoint.z;
        cloud->push_back(p);
    }

    pcl::io::savePLYFileBinary(filename, *cloud);

}