#ifndef JACO2_CALIBRATION_IO_H
#define JACO2_CALIBRATION_IO_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <jaco2_data/dynamic_calibrated_parameters.hpp>
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/accelerometer_data.h>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
//#include <yaml-cpp/yaml.h>
namespace Jaco2Calibration {

class Jaco2CalibrationIO{


    Jaco2CalibrationIO();

public:
    static void save(std::string name, const DynamicParametersCollection& params);

    static void loadDynParm(std::string filename, DynamicParametersCollection& params);

    static void save(std::string name, jaco2_data::JointStateDataStampedCollection& samples, std::string delimiter = std::string(";"));
    static void save(std::string filename, jaco2_data::AccelerometerDataCollection &samples, std::string delimiter = std::string(";"));

    static void importAsciiData(std::string filename, jaco2_data::JointStateDataStampedCollection &samples, const char delimiter = ';');

    static void importAsciiDataWithGravity(std::string filename, jaco2_data::JointStateDataStampedCollection&  samples, const char delimiter = ';');

    static void importAsciiData(std::string filename, AccelerationSamples& samples, const char delimiter = ';');

};
}
#endif // JACO2_CALIBRATION_IO_H

