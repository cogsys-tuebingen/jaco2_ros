#ifndef DYNAMIC_CALIBRATED_PARAMETERS_HPP
#define DYNAMIC_CALIBRATED_PARAMETERS_HPP
#include <string>
#include <tf/tf.h>
namespace Jaco2Calibration {

struct DynamicCalibratedParameters
{
    std::string linkName;
    double mass;
    tf::Vector3 coM;
    tf::Matrix3x3 inertia;

};
}
#endif // DYNAMIC_CALIBRATED_PARAMETERS_HPP

