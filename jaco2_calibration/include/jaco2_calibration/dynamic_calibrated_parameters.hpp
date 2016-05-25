#ifndef DYNAMIC_CALIBRATED_PARAMETERS_HPP
#define DYNAMIC_CALIBRATED_PARAMETERS_HPP

#include <tf/tf.h>

struct DynamicCalibratedParameters
{

    tf::Vector3 coM;
    tf::Matrix3x3 inertia;

};

#endif // DYNAMIC_CALIBRATED_PARAMETERS_HPP

