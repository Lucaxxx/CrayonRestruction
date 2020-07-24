#ifndef CRAYONRESTRUCTION_VIEWER_H
#define CRAYONRESTRUCTION_VIEWER_H


#include <vtkAutoInit.h>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "mappoint.h"


class Viewer{
public:
    void display(const vector<MapPoint>& mappoints);
};



#endif //CRAYONRESTRUCTION_VIEWER_H
