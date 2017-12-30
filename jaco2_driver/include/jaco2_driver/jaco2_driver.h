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
#include <jaco2_driver/joint_trajectory.h>
#include <jaco2_data/accelerometer_calibration.hpp>
#include <jaco2_data/torque_offset_lut.hpp>
#include <jaco2_data/gravity_params.hpp>
#include <jaco2_data/torque_offset_calibration.hpp>
#include <jaco2_driver/jaco2_driver_configureConfig.h>
#include <jaco2_driver/utility/delegate.hpp>
//Jaco2 Controller
#include <jaco2_driver/controller/jaco2_controller.h>
#include <jaco2_driver/controller/empty_controller.h>
#include <jaco2_driver/controller/angular_position_controller.h>
#include <jaco2_driver/controller/gripper_controller.h>
#include <jaco2_driver/controller/gravity_compensation_controller.hpp>
#include <jaco2_driver/controller/torque_controller.h>
#include <jaco2_driver/controller/controller_factory.hpp>



class Jaco2Driver
{

public:
    Jaco2Driver();
    ~Jaco2Driver();

    bool controllerFinished(Jaco2Controller::Result &result_type) const;
    bool serviceDone() const;
    bool initialize(std::string serial = std::string(""), bool right = true, bool move_home = true);
    // GET
    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularAcceleration() const;
    AngularPosition getAngularForce() const;
    AngularPosition getCurrentTrajError() const;
    AngularPosition getAngularForceGravityFree() const;
    AngularPosition getCurrent() const;
    AngularAcceleration getActuatorAcceleration() const;
    QuickStatus getQuickStatus() const;
    SensorsInfo getSensorInfo() const;
    jaco2_data::TimeStamp getLastReadUpdate(int read_data) const;
    jaco2_data::JointStateDataStamped getJointState() const;
    jaco2_data::AccelerometerData getAccelerometerData() const;
    Jaco2Calibration::TorqueOffsetLut getTorqueCalibration() const;

    int getSetTorqueZeroResult() const;

    //SET
    void setAngularPosition(const AngularPosition &position);
    void setAngularVelocity(const AngularPosition &velocity);
    void setFingerVelocity(const AngularPosition &finger_velocity);
    void setFingerPosition(const AngularPosition & position);
    void setTrajectory(const JointTrajectory & trajectory);
    void stop();
    void stopMovement();

    void setStatePriorityRatio(const int r);
    void setStateHighPriorityQue(const std::vector<int> &que);
    void setStateLowPriorityQue(const std::vector<int> &que);
    void setTorque(const AngularPosition& torque);
    void enableGravityCompensation();
    void disableGravityCompensation();

    void grabObj(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3);
    void grabObjSetUnusedFingerPos(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3, const int posFinger1, const int posFinger2, const int posFinger3);
    void setAccelerometerCalibration(const std::vector<Jaco2Calibration::AccelerometerCalibrationParam>& params);
    void setTorqueCalibration(const Jaco2Calibration::TorqueOffsetLut& lut);
    void setTorqueCalibration(const Jaco2Calibration::TorqueOffsetCalibration& lut);
    bool setGravityParams(const Jaco2Calibration::ApiGravitationalParams &params);
    void setVelocitySensorCalibration(const std::vector<double>& factors);

    void enableForceControl();
    void disableForceControl();

    void setJointNames(const std::vector<std::string>& names);



// BEGIN EXPERIMENTAL
    void setVelocityController(const std::string& type);
    void setTrajectoryController(const std::string& type);

    void updateControllerConfig(jaco2_driver::jaco2_driver_configureConfig& cfg);

// END EXPERIMENTAL

    void startArm();
    void stopArm();
    void homeArm();
    void setTorqueZero(int actuator);

    void finish();

    unsigned char getRobotType(){return quickStatus_.RobotType;}

    static const int U_SlEEP_TIME = 5000;

private:
    void setActiveController(Jaco2Controller* controller);

    void controllerTerminated(const Jaco2Controller::Result res);

private:
    Jaco2Controller::TerminationCallback t_;
    bool initialized_;
    bool controller_done_;
    Jaco2API jaco_api_;

    std::string serial_;
    bool right_arm_;
    Jaco2Controller* active_controller_;
    Jaco2Controller::Result result_;
    AngularPositionController position_controller_;
    EmptyController empty_controller_;
    GripperController gripper_controller_;
    GravityCompensationController gravity_comp_controller_;
    TorqueController torque_controller_;
    // Variable Controller
    std::shared_ptr<VelocityController> velocity_controller_;
    std::shared_ptr<TrajectoryTrackingController> trajectory_controller_;

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
    int setTorqueZeroResult_;
    bool paused_;
    bool serviceDone_;


};

#endif // JACO2DRIVER_H
