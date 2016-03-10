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
    Jaco2API(void);
    ~Jaco2API();

    int init();

    AngularPosition getAngularVelocity() const;
    void setAngularVelocity(const TrajectoryPoint &velocity);

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
    int (*GetAngularVelocity)(AngularPosition &);


private:
    mutable std::recursive_mutex mutex_;
};

#endif // JACO2API_H
