#ifndef ABSTRACT_TRAJECTORY_TRACKING_CONTROLLER_H
#define ABSTRACT_TRAJECTORY_TRACKING_CONTROLLER_H

#include <jaco2_driver/controller/jaco2_controller.h>
#include <jaco2_driver/joint_trajectory.h>

class TrajectoryTrackingController : public Jaco2Controller
{
public:
    TrajectoryTrackingController(Jaco2State &state, Jaco2API &api)
        : Jaco2Controller(state, api)
    {

    }
    virtual bool isDone() const = 0;
    virtual void start() = 0;

    virtual void setTrajectory(const JointTrajectory& trajectory) = 0;
    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) = 0;

protected:
    virtual void write()= 0;
};
#endif // ABSTRACT_TRAJECTORY_TRACKING_CONTROLLER_H
