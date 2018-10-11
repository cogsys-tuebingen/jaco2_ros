#ifndef POINT_2_POINT_VELOCITY_CONTROLLER_H
#define POINT_2_POINT_VELOCITY_CONTROLLER_H

#include <jaco2_driver/controller/p2p_joint_trajectory_controller.h>
#include <jaco2_driver/controller/trajectory_tracking_controller.h>

class Point2PointVelocityController : public TrajectoryTrackingController
{
public:
    Point2PointVelocityController(Jaco2State &state, Jaco2API& api, TerminationCallback &t);

    virtual void write() override;
    virtual void start() override;
    virtual bool isDone() const;

    void setTrajectory(const JointTrajectory& trajectory) override;
    AngularInfo getJointError() const override;

    void setGainP(const ManipulatorInfo& gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);
    void setConfig(jaco2_driver::jaco2_driver_configureConfig &cfg) override;
    void stopMotion();

protected:
    void pidController(const double dt);
    void simpleVelController(const double dt);

protected:
    P2PJointTrajactoryControllerHelper trajectory_wrapper_;
    TrajectoryPoint tp_;
    ManipulatorInfo gainP_;
    ManipulatorInfo gainI_;
    ManipulatorInfo gainD_;





};
#endif // POINT_2_POINT_VELOCITY_CONTROLLER_H

