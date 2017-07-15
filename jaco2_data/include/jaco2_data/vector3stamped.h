#ifndef VECTOR3STAMPED_H
#define VECTOR3STAMPED_H
#include <jaco2_data/time_stamp.h>
#include <Eigen/Dense>

namespace jaco2_data {

class Vector3Stamped
{
public:
    Vector3Stamped();

public:
    TimeStamp stamp;
    std::string frame_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d vector;

};
}
#endif // VECTOR3STAMPED_H
