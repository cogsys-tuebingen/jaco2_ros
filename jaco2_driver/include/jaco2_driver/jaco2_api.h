#ifndef JACO2API_H
#define JACO2API_H

#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.CommLayerUbuntu.h>
#include <kinova/Kinova.API.UsbCommandLayerUbuntu.h>
#include <mutex>

enum ActuatorID{
    Actuator1 = 1,
    Actuator2 = 2,
    Actuator3 = 3,
    Actuator4 = 4,
    Actuator5 = 5,
    Actuator6 = 6
};

class Jaco2API
{
public:
    Jaco2API();
    ~Jaco2API();

    int init(std::string serial = "", bool right = true);

    bool  isStopped() const;
    QuickStatus getQuickStatus() const;
    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularForce() const;
    AngularPosition getAngularForceGravityFree() const;
    AngularPosition getAngularCurrent() const;
    AngularAcceleration getActuatorAcceleration() const;
    SensorsInfo getSensorInfo() const;
    void setAngularVelocity(const TrajectoryPoint &velocity);
    void setAngularPosition(const TrajectoryPoint &position);
    void setAngularTorque(const AngularPosition &torque);
    void setAngularTorque(const AngularInfo &torque);
    void setLimitedAngularCmd(const TrajectoryPoint &point);
    void startAPI();
    void stopAPI();
    void exitAPI();
    void moveHome();
    void initFingers();
    /**
     * @brief setTorqueZero sets the actuator torque sensor to zero for current position
     * @param actuator the index of the actuator [1,..., 6]
     */
    int setTorqueZero(ActuatorID actuator);
    void enableDirectTorqueMode(double torque_saftey_factor = 0.6, double vibration_controller = 0.5);
    void getApiVersion(int& v_major, int& v_minor, int& version);
    /**
     * @brief inTrajectoryMode
     * @return true: Trajectory mode. false: Torque mode.
     */
    bool inTrajectoryMode();

    void disableTorque();

    bool setGravityOptimalZParam(const std::vector<double>& params);

    void runGravityEstimationSequnce(std::vector<double>& res, ROBOT_TYPE type = JACOV2_6DOF_SERVICE);
    void setGravityType(GRAVITY_TYPE type);
    /**
     * @brief setActuatorPID sets the PID values of a controller of an acctuator
     * @param actuator number of an actuator between 1 and 6
     * @param p porotional controller gain
     * @param i integrative controller gain
     * @param d derivative controller gain
     */
    void setActuatorPID(ActuatorID actuator, double p, double i, double d);

    void startForceControl();
    void stopForceControl();


private:
    void * commandLayer_handle;

    int (*InitAPI)();
    int (*CloseAPI)();
    int (*StopControlAPI)();
    int (*StartControlAPI)();
    int (*SendBasicTrajectory)(TrajectoryPoint command);
    int (*GetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
    int (*SetActiveDevice)(KinovaDevice device);
    int (*MoveHome)();
    int (*InitFingers)();
    int (*GetAngularCommand)(AngularPosition &);
    int (*GetAngularPosition)(AngularPosition &);
    int (*GetAngularVelocity)(AngularPosition &);
    int (*GetAngularForce)(AngularPosition &Response);
    int (*GetAngularForceGravityFree)(AngularPosition &Response);
    int (*GetQuickStatus)(QuickStatus &Response);
    int (*GetAngularCurrent)(AngularPosition &);
    int (*EraseAllTrajectories)();
    int (*GetActuatorAcceleration)(AngularAcceleration & Response);
    int (*SetAngularControl)();
    int (*SetCartesianControl)();
    int (*GetSensorsInfo)(SensorsInfo &);
    int (*SetTorqueZero)(int);
    int (*SendAdvanceTrajectory)(TrajectoryPoint command);          /// Send Trajectory with Limits
    int (*StopCurrentLimitation)();
    int (*GetCartesianPosition)(CartesianPosition &);
    int (*GetAPIVersion)(int Response[API_VERSION_COUNT]);
    int(*RunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
    int(*SwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    int(*SetTorqueSafetyFactor)(float factor);
    int(*SendAngularTorqueCommand)(float Command[COMMAND_SIZE]);
    int(*SendCartesianForceCommand)(float Command[COMMAND_SIZE]);
    int(*SetGravityVector)(float Command[3]);
    int(*SetGravityPayload)(float Command[GRAVITY_PAYLOAD_SIZE]);
    int(*SetGravityOptimalZParam)(float Command[GRAVITY_PARAM_SIZE]);
    int(*SetGravityType)(GRAVITY_TYPE Type);
    int(*SetTorqueVibrationController)(float value);
    int(*GetCartesianForce)(CartesianPosition &);
    int(*SetTorqueControlType)(TORQUECONTROL_TYPE type);
    int(*GetTrajectoryTorqueMode)(int &);
    int(*SetActuatorPID)(unsigned int address, float P, float I, float D);
    int (*StartForceControl)();
    int (*StopForceControl)();


    void moveHomeLeft();
private:
    mutable std::recursive_mutex mutex_;
    bool stopedAPI_;
    bool right_arm_;
};

#endif // JACO2API_H
