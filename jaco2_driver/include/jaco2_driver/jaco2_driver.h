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

class Jaco2Driver
{
public:
    Jaco2Driver();
    ~Jaco2Driver();

    AngularPosition getAngularVelocity() const;
    void setAngularVelocity(const TrajectoryPoint &velocity);

    void stop();

private:
    void tick();

private:
    Jaco2API jaco_api_;

    std::vector<double> jointAngles_;
    std::vector<double> jointVelocities_;
    std::vector<double> jointEffort_;

    void getJointValues();

private:
    std::thread spinner_;
    mutable std::recursive_mutex running_mutex_;
    bool running_;

    mutable std::recursive_mutex data_mutex_;
    TrajectoryPoint target_velocity_;
    AngularPosition current_velocity_;
};

#endif // JACO2DRIVER_H
