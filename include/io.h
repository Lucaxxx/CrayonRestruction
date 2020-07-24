#ifndef CRAYONRESTRUCTION_IO_H
#define CRAYONRESTRUCTION_IO_H

#include "mappoint.h"
#include <pcl/io/ply_io.h>

using std::string;
using std::map;
using std::vector;
using std::cout;
using std::endl;


class io{
public:
    void write(const string& filename, const vector<MapPoint>& mappoints);
};


#endif