#ifndef JACO2CALIBRATION_H
#define JACO2CALIBRATION_H
//System
#include <vector>
#include <ceres/ceres.h>
//ROS

//Jaco2 ROS
#include <jaco2_kin_dyn/jaco2_kinematics_dynamics.h>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>
#include <jaco2_calibration/dynamic_calibration_sample.hpp>
#include <jaco2_calibration/dynamic_calibrated_parameters.hpp>

class Jaco2Calibration
{
public:
    Jaco2Calibration(std::string& urdf_param, std::string& root, std::string& tip);
    ~Jaco2Calibration();

    int calibrateCoMandInertia(const std::vector<DynamicCalibrationSample> &samples);

    std::vector<DynamicCalibratedParameters> getDynamicCalibration() const { return dynParams_;}

private:
    Jaco2KinematicsDynamicsModel model_;
    std::vector<DynamicCalibratedParameters> dynParams_;
};

#endif // JACO2CALIBRATION_H
