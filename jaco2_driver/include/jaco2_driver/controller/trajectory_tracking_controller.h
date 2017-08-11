#ifndef TRAJECTORY_TRACKING_CONTROLLER_H
#define TRAJECTORY_TRACKING_CONTROLLER_H

#include <jaco2_driver/controller/jaco2_controller.h>
#include <jaco2_driver/joint_trajectory.h>
#include <jaco2_driver/manipulator_info.h>
#include <jaco2_driver/controller/p2p_joint_trajectory_controller.h>

class TrajectoryTrackingController : public Jaco2Controller
{
public:
    TrajectoryTrackingController(Jaco2State &state, Jaco2API &api, TerminationCallback &t);


    virtual bool isDone() const = 0;
    virtual void start() = 0;

    virtual void setTrajectory(const JointTrajectory& trajectory) = 0;

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) = 0;
    virtual AngularInfo getJointError() const = 0;

protected:
    virtual void write()= 0;

protected:
    std::size_t current_point_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_command_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;

};
#endif // TRAJECTORY_TRACKING_CONTROLLER_H
