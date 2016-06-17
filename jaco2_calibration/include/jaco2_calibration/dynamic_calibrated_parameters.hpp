#ifndef DYNAMIC_CALIBRATED_PARAMETERS_HPP
#define DYNAMIC_CALIBRATED_PARAMETERS_HPP
#include <string>
//#include <tf/tf.h>
#include <Eigen/Core>
namespace Jaco2Calibration {

struct DynamicCalibratedParameters
{
    std::string linkName;
    double mass;
    Eigen::Vector3d coM;
    Eigen::Matrix3d inertia;

};
}
#endif // DYNAMIC_CALIBRATED_PARAMETERS_HPP

