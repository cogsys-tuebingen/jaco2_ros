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
#include <jaco2_driver/jaco2_api.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_driver/jaco2_driver_command.h>

class Jaco2Driver
{
public:
    Jaco2Driver();
    ~Jaco2Driver();

    bool reachedAngularGoal() const;
    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularForce() const;
    void setAngularPosition(const AngularPosition &position);
    void setAngularVelocity(const AngularPosition &velocity);
    void stop();
    void stopMovement();

    unsigned char getRobotType(){return quickStatus_.RobotType;}

    static const int U_SlEEP_TIME = 5000;

private:
    Jaco2API jaco_api_;

    std::vector<double> jointAngles_;
    std::vector<double> jointVelocities_;
    std::vector<double> jointEffort_;

    QuickStatus quickStatus_;

    void getJointValues();
    void tick();
    bool reachedAngularGoal(const TrajectoryPoint &goal);

private:
    std::thread spinner_;
    mutable std::recursive_mutex running_mutex_;
    bool running_;

    mutable std::recursive_mutex data_mutex_;
    AngularPosition current_position_;
    AngularPosition current_velocity_;
    AngularPosition current_torque_;
    std::time_t last_command_;

    int readCmd_;
    std::function<void()> write_;
    std::function<void()> read_;

    bool moveToAngularPos_;
    bool reachedAngularPos_;
    std::vector<TrajectoryPoint> trajQueue_;


};

#endif // JACO2DRIVER_H
