#ifndef POINT_2_POINT_VELOCITY_CONTROLLER_H
#define POINT_2_POINT_VELOCITY_CONTROLLER_H

#include "velocity_controller.h"
#include "joint_trajectory.h"
#include "manipulator_info.h"
#include <jaco2_driver/jaco2_api.h>

class Point2PointVelocityController : public VelocityController
{
public:
    Point2PointVelocityController(Jaco2State &state, Jaco2API& api);

    void setTrajectory(const JointTrajectory& trajectory);

    virtual void write() override;

    virtual bool isDone() const;

private:
    JointTrajectory trajectory_;
    std::size_t  current_point_;
    std::vector<ManipulatorInfo> paramsConst_;
    std::vector<ManipulatorInfo> paramsLinear_;
    std::vector<ManipulatorInfo> paramsSquare_;
    std::vector<ManipulatorInfo> paramsCube_;
    std::vector<ManipulatorInfo> paramsQuad_;
    std::vector<double> timeDiff_;

    TrajectoryPoint tp_;

    std::time_t start_command_;

    bool done_;

    double jointCmdVelocity(const double dt, const std::size_t joint);

};
#endif // POINT_2_POINT_VELOCITY_CONTROLLER_H

