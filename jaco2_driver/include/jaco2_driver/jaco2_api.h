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

    QuickStatus getQuickStatus() const;
    AngularPosition getAngularPosition() const;
    AngularPosition getAngularVelocity() const;
    AngularPosition getAngularForce() const;
    AngularPosition getAngularForceGravityFree() const;
    AngularPosition getAngularCurrent() const;
    void setAngularVelocity(const TrajectoryPoint &velocity);
    void setAngularPosition(const TrajectoryPoint &position);

private:
    void * commandLayer_handle;

    int (*InitAPI)();
    int (*CloseAPI)();
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



private:
    mutable std::recursive_mutex mutex_;
};

#endif // JACO2API_H
