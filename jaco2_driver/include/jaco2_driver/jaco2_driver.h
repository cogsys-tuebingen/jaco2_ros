#ifndef JACO2DRIVER_H
#define JACO2DRIVER_H
//System
#include <vector>
#include <thread>
#include <mutex>
// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//Jaco2
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_driver/jaco2_controller.h>
#include <jaco2_driver/angular_position_controller.h>
#include <jaco2_driver/joint_trajectory.h>
//Jaco2 Controller
#include <jaco2_driver/jaco2_controller.h>
#include <jaco2_driver/empty_controller.h>
#include <jaco2_driver/angular_position_controller.h>
#include <jaco2_driver/velocity_controller.h>
#include <jaco2_driver/point_2_point_velocity_controller.h>
#include <jaco2_driver/gripper_pid_controller.h>

class Jaco2Driver
{
public:
    Jaco2Driver();
    ~Jaco2Driver();

    bool reachedGoal() const;

    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularForce() const;
    AngularPosition getCurrentTrajError() const;
    void setAngularPosition(const AngularPosition &position);
    void setAngularVelocity(const AngularPosition &velocity);
    void setFingerPosition(const AngularPosition & position);
    void setTrajectory(const JointTrajectory & trajectory);
    void stop();
    void stopMovement();
    void setTrajectoryPGains(const ManipulatorInfo& gains);
    void setTrajectoryIGains(const ManipulatorInfo& gains);
    void setTrajectoryDGains(const ManipulatorInfo& gains);
    void setGripperEffort(const double effort);
    void setGripperPGain(const double finger1, const double finger2, const double finger3);
    void setGripperIGain(const double finger1, const double finger2, const double finger3);
    void setGripperDGain(const double finger1, const double finger2, const double finger3);

    void finish();

    unsigned char getRobotType(){return quickStatus_.RobotType;}

    static const int U_SlEEP_TIME = 5000;

private:
    Jaco2API jaco_api_;

    Jaco2Controller* active_controller_;

    VelocityController velocity_controller_;
    AngularPositionController position_controller_;
    Point2PointVelocityController p2p_velocity_controller_;
    EmptyController empty_controller_;
    GripperPIDController gripper_controller_;

    std::vector<double> jointAngles_;
    std::vector<double> jointVelocities_;
    std::vector<double> jointEffort_;

    QuickStatus quickStatus_;

    void getJointValues();
    void tick();

private:
    std::thread spinner_;
    mutable std::recursive_mutex running_mutex_;
    bool running_;

    Jaco2State state_;



};

#endif // JACO2DRIVER_H
