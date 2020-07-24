#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "tracker.h"
#include "viewer.h"
#include "io.h"

const std::string camerayaml="/home/lucas/projects/CrayonRestruction/config/camera.yaml";
const std::string pictureyaml="/home/lucas/projects/CrayonRestruction/config/config.yaml";
const std::string pwd="/home/lucas/projects/CrayonRestruction/data/pcloud.ply";

int main(){

    Tracker tracker(camerayaml,pictureyaml);

    tracker.Tracking();

    sleep(5);

    Viewer v;
    v.display(tracker.mvMapPoints);

    io output;
    output.write(pwd,tracker.mvMapPoints);

    return 0;
}