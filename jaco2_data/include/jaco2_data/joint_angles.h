#ifndef JOINT_ANGLES_H
#define JOINT_ANGLES_H
#include <vector>
#include <jaco2_data/joint_data.h>

namespace jaco2_data {

class JointAngles : public JointData
{

public:
    JointAngles();
    JointAngles(std::size_t n);
    JointAngles(const JointData& d);

    JointAngles normalize() const;
    JointAngles& operator =(const JointData &other);

private:
    static double normalize(double angle);
    bool normalized_;


};
}
#endif // JOINT_ANGLES_H
