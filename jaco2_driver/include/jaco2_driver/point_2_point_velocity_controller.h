#ifndef POINT_2_POINT_VELOCITY_CONTROLLER_H
#define POINT_2_POINT_VELOCITY_CONTROLLER_H

#include "velocity_controller.h"
#include "joint_trajectory.h"
#include "manipulator_info.h"
#include <jaco2_driver/jaco2_api.h>
#include <chrono>

class Point2PointVelocityController : public VelocityController
{
public:
    Point2PointVelocityController(Jaco2State &state, Jaco2API& api);

    void setTrajectory(const JointTrajectory& trajectory);
    void setGainP(const ManipulatorInfo &gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);
    AngularInfo getJointError() const;

    virtual void write() override;
    virtual void start() override;

    virtual bool isDone() const;

    double jointAcceleration(const double dt, const std::size_t joint) const;
    double jointCmdVelocity(const double dt, const std::size_t joint) const;
    double jointPosition(const double dt, const std::size_t joint) const;

protected:
    ManipulatorInfo diffTrajectoryPoint();
    void pidController(const double dt);
    void simpleVelController(const double dt);

    void evaluationOutput();

protected:
    JointTrajectory trajectory_;
    std::size_t  current_point_;
    ManipulatorInfo gainP_;
    ManipulatorInfo gainI_;
    ManipulatorInfo gainD_;
    std::vector<ManipulatorInfo> paramsConst_;
    std::vector<ManipulatorInfo> paramsLinear_;
    std::vector<ManipulatorInfo> paramsSquare_;
    std::vector<ManipulatorInfo> paramsCube_;

    ManipulatorInfo eLast_;
    ManipulatorInfo eSum_;
    double samplingPeriod_;
    std::vector<double> timeDiff_;
    std::vector<ManipulatorInfo> posDiff_;

    TrajectoryPoint tp_;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_command_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;




};
#endif // POINT_2_POINT_VELOCITY_CONTROLLER_H

