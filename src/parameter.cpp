#include "calibration.h"


const std::string inputyaml="/home/lucas/projects/CrayonRestruction/config/config.yaml";
const std::string outputyaml="/home/lucas/projects/CrayonRestruction/config/camera.yaml";


int main(){


    Calibration calibrator(inputyaml);

    calibrator.Calibrate();

    calibrator.print();

    calibrator.write(outputyaml);


    return 0;
}