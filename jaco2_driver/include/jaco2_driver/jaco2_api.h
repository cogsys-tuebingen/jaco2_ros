#ifndef JACO2API_H
#define JACO2API_H

#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <kinova/KinovaTypes.h>
#include <kinova/Kinova.API.CommLayerUbuntu.h>
#include <mutex>

class Jaco2API
{
public:
    Jaco2API();
    ~Jaco2API();

    int init();

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
    void startAPI();
    void stopAPI();
    void moveHome();
    void initFingers();
    /**
     * @brief setTorqueZero sets the actuator torque sensor to zero for current position
     * @param actuator the index of the actuator [1,..., 6]
     */
    int setTorqueZero(int actuator);

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



private:
    mutable std::recursive_mutex mutex_;
    bool stopedAPI_;
};

#endif // JACO2API_H
