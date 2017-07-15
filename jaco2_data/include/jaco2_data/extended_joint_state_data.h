#ifndef EXTENDED_JOINT_STATE_DATA_H
#define EXTENDED_JOINT_STATE_DATA_H
#include <jaco2_data/joint_state_data.h>
#include <jaco2_data/accelerometer_data.h>
namespace jaco2_data {
class ExtendedJointStateData
{
public:
    ExtendedJointStateData() {}

public:
    JointStateData joint_state;
    AccelerometerData lin_acc;
};
}
#endif // EXTENDED_JOINT_STATE_DATA_H
