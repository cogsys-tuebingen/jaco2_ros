#ifndef JACO2_CALIBRATION_IO_H
#define JACO2_CALIBRATION_IO_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <jaco2_calibration_utils/dynamic_calibrated_parameters.hpp>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
//#include <yaml-cpp/yaml.h>
namespace Jaco2Calibration {

class Jaco2CalibrationIO{


    Jaco2CalibrationIO();

public:
    static void save(std::string name, const std::vector<DynamicCalibratedParameters>& params);

    static void loadDynParm(std::string filename, std::vector<DynamicCalibratedParameters>& params);

    static void save(std::string name, std::vector<DynamicCalibrationSample> samples, std::string delimiter = std::string(";"));

    static void importAsciiData(std::string filename, std::vector<DynamicCalibrationSample>& samples, const char delimiter = ';');

    static void importAsciiDataWithGravity(std::string filename, std::vector<DynamicCalibrationSample>& samples, const char delimiter = ';');

    static void importAsciiData(std::string filename, AccelerationSamples& samples, const char delimiter = ';');

};
}
#endif // JACO2_CALIBRATION_IO_H

