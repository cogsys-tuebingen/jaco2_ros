#ifndef P2P_JOINT_TRAJECTORY_CONTROLLER_H
#define P2P_JOINT_TRAJECTORY_CONTROLLER_H

#include <jaco2_driver/controller/trajectory_tracking_controller.h>
#include <jaco2_driver/manipulator_info.h>
#include <jaco2_driver/joint_trajectory.h>

class P2PJointTrajactoryController : public TrajectoryTrackingController
{
public:
    P2PJointTrajactoryController(Jaco2State &state, Jaco2API &api);

    virtual ~P2PJointTrajactoryController() = default;

    virtual bool isDone() const = 0;

    virtual void start() = 0;

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override;

    void setTrajectory(const JointTrajectory& trajectory);
    void setGainP(const ManipulatorInfo &gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);
    AngularInfo getJointError() const;

    double jointAcceleration(const double dt, const std::size_t joint) const;
    double jointCmdVelocity(const double dt, const std::size_t joint) const;
    double jointPosition(const double dt, const std::size_t joint) const;

protected:
    virtual void write() = 0;


protected:
    ManipulatorInfo diffTrajectoryPoint();

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
#endif // P2P_JOINT_TRAJECTORY_CONTROLLER_H
