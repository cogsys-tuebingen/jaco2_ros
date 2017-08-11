#ifndef P2P_JOINT_TRAJECTORY_CONTROLLER_H
#define P2P_JOINT_TRAJECTORY_CONTROLLER_H

#include <jaco2_driver/manipulator_info.h>
#include <jaco2_driver/joint_trajectory.h>

class P2PJointTrajactoryControllerHelper
{
public:
    P2PJointTrajactoryControllerHelper();


    void setTrajectory(const JointTrajectory& trajectory);
    AngularInfo getJointError() const;

    void clear();
    double jointAcceleration(const double dt, const std::size_t joint) const;
    double jointVelocity(const double dt, const std::size_t joint) const;
    double jointPosition(const double dt, const std::size_t joint) const;
    void setPositionDifference(const AngularInfo& current_position);


//protected:
    ManipulatorInfo diffTrajectoryPoint(const AngularInfo& current_position);

    void evaluationOutput();

//protected:
    JointTrajectory trajectory_;
    std::size_t  current_point_;

    std::vector<ManipulatorInfo> paramsConst_;
    std::vector<ManipulatorInfo> paramsLinear_;
    std::vector<ManipulatorInfo> paramsSquare_;
    std::vector<ManipulatorInfo> paramsCube_;

    ManipulatorInfo eLast_;
    ManipulatorInfo eSum_;
    double samplingPeriod_;
    std::vector<double> timeDiff_;
    std::vector<ManipulatorInfo> posDiff_;

//    TrajectoryPoint tp_;


};
#endif // P2P_JOINT_TRAJECTORY_CONTROLLER_H
