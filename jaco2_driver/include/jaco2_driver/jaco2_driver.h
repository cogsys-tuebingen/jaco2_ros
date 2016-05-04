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
#include <kinova/KinovaTypes.h>
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
#include <jaco2_driver/gripper_controller.h>

class Jaco2Driver
{
public:
    Jaco2Driver();
    ~Jaco2Driver();

    bool reachedGoal() const;
    // GET
    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularForce() const;
    AngularPosition getCurrentTrajError() const;
    AngularPosition getAngularForceGravityFree() const;
    AngularPosition getCurrent() const;
    AngularAcceleration getActuatorAcceleration() const;
    QuickStatus getQuickStatus() const;
    SensorsInfo getSensorInfo() const;
    std::chrono::time_point<std::chrono::high_resolution_clock> getLastReadUpdate(int read_data) const;
    //SET
    void setAngularPosition(const AngularPosition &position);
    void setAngularVelocity(const AngularPosition &velocity);
    void setFingerVelocity(const AngularPosition &finger_velocity);
    void setFingerPosition(const AngularPosition & position);
    void setTrajectory(const JointTrajectory & trajectory);
    void stop();
    void stopMovement();
    void setTrajectoryPGains(const ManipulatorInfo& gains);
    void setTrajectoryIGains(const ManipulatorInfo& gains);
    void setTrajectoryDGains(const ManipulatorInfo& gains);
    void setStatePriorityRatio(const int r);

    void setGripperPGain(const double finger1, const double finger2, const double finger3);
    void setGripperIGain(const double finger1, const double finger2, const double finger3);
    void setGripperDGain(const double finger1, const double finger2, const double finger3);
    void setGripperFingerVelocity(const int finger1, const int finger2, const int finger3);

    void grabObj(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3);
    void grabObjSetUnusedFingerPos(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3, const int posFinger1, const int posFinger2, const int posFinger3);

    void startArm();
    void stopArm();
    void homeArm();

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
    GripperController gripper_controller_;

    std::vector<double> jointAngles_;
    std::vector<double> jointVelocities_;
    std::vector<double> jointEffort_;

    QuickStatus quickStatus_;

    void getJointValues();
    void tick();
    void executeLater(std::function<void()> fn);

private:
    std::thread spinner_;
    mutable std::recursive_mutex running_mutex_;
    bool running_;

    Jaco2State state_;

    std::vector<std::function<void()>> commands_;
    mutable std::recursive_mutex commands_mutex_;
    bool paused_;

};

#endif // JACO2DRIVER_H
