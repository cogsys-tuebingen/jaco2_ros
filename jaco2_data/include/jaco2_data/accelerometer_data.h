#ifndef ACCELEROMETER_DATA_H
#define ACCELEROMETER_DATA_H
#include <jaco2_data/vector3stamped.h>
namespace jaco2_data {

class AccelerometerData
{
public:
    AccelerometerData()
        : user_defined_label(-1)
    {}

public:
    int user_defined_label;
    std::vector<Vector3Stamped> lin_acc;
};
}
#endif // ACCELEROMETER_DATA_H
