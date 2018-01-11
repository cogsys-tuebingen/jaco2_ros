#ifndef TORQUE_TRAJECTORY_CONTROLLER_H
#define TORQUE_TRAJECTORY_CONTROLLER_H

#include <jaco2_driver/controller/trajectory_tracking_controller.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
class TorqueTrajectoryController  : public TrajectoryTrackingController
{
public:
    TorqueTrajectoryController(Jaco2State &state, Jaco2API& api, TerminationCallback& t);

    void setTrajectory(const JointTrajectory& trajectory) override;

    void write() override;
    void start() override;
    bool isDone() const;
    virtual void stop() override;
    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override;

    void control(const double dt);

    AngularInfo getJointError() const;

    void setGainP(const ManipulatorInfo& gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);



protected:
    void getDesiredPosition(double dt, AngularInfo& result);
    void getDesiredVelocity(double dt, AngularInfo& result);
    void getDesiredAcceleration(double dt,  AngularInfo& result);
    void getDesiredPosition(double dt, std::vector<double>& result);
    void getDesiredVelocity(double dt, std::vector<double>& result);
    void getDesiredAcceleration(double dt, std::vector<double>& result);


protected:
    P2PJointTrajactoryControllerHelper trajectoryWrapper_;
    Jaco2KinDynLib:: Jaco2DynamicModel model_;
    AngularInfo cmd_;
    ManipulatorInfo gainP_;
    ManipulatorInfo gainI_;
    ManipulatorInfo gainD_;

    // Debug
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub2;
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState joint_state2;
};

#endif // TORQUE_TRAJECTORY_CONTROLLER_H
