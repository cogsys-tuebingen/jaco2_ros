#ifndef TORQUE_TRAJECTORY_CONTROLLER_H
#define TORQUE_TRAJECTORY_CONTROLLER_H

#include <jaco2_driver/controller/point_2_point_velocity_controller.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
class TorqueTrajectoryController  : public Point2PointVelocityController
{
public:
    TorqueTrajectoryController(Jaco2State &state, Jaco2API& api);

    void setTrajectory(const JointTrajectory& trajectory);
//    void setGainP(const ManipulatorInfo &gains);
//    void setGainI(const ManipulatorInfo &gains);
//    void setGainD(const ManipulatorInfo &gains);
//    AngularInfo getJointError() const;

    void write() override;
    void start() override;
    bool isDone() const;
    virtual void stop() override;

    void control(const double dt);

protected:
    void getDesiredPosition(double dt, AngularInfo& result);
    void getDesiredVelocity(double dt, AngularInfo& result);
    void getDesiredAcceleration(double dt,  AngularInfo& result);
    void getDesiredPosition(double dt, std::vector<double>& result);
    void getDesiredVelocity(double dt, std::vector<double>& result);
    void getDesiredAcceleration(double dt, std::vector<double>& result);


protected:
    Jaco2KinDynLib:: Jaco2DynamicModel model_;

    // Debug
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub2;
    sensor_msgs::JointState joint_state;
    sensor_msgs::JointState joint_state2;
};

#endif // TORQUE_TRAJECTORY_CONTROLLER_H
