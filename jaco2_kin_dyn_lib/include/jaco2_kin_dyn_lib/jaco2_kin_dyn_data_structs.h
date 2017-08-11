#ifndef JACO2_KIN_DYN_DATA_STRUCTS_H
#define JACO2_KIN_DYN_DATA_STRUCTS_H
#include <vector>

namespace Jaco2KinDynLib {

struct ResidualData
{
    double dt;
    double gx;
    double gy;
    double gz;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> torques;
};

struct JointState{

    double dt;
    double gx;
    double gy;
    double gz;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_accelerations;
    std::vector<double> torques;

    ResidualData getResidualData()
    {
        ResidualData res;
        res.dt = dt;
        res.gx = gx;
        res.gy = gy;
        res.gz = gz;
        res.joint_positions = joint_positions;
        res.joint_velocities = joint_velocities;
        res.torques = torques;
        return res;
    }

};


}
#endif // JACO2_KIN_DYN_DATA_STRUCTS_H

