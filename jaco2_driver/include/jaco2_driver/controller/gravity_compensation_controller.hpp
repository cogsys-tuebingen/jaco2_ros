#ifndef GRAVITY_COMPENSATION_CONTROLLER_HPP
#define GRAVITY_COMPENSATION_CONTROLLER_HPP
#include <ctime>
#include "jaco2_controller.h"
#include <kinova/KinovaTypes.h>
class GravityCompensationController : public Jaco2Controller
{
public:
    GravityCompensationController(Jaco2State &state, Jaco2API &api)
        : Jaco2Controller(state, api),
          last_command_(std::time(nullptr)),
          done_(false)
    {
        tp_.InitStruct();
        tp_.Position.Type = ANGULAR_VELOCITY;
    }

    virtual void start() override
    {
        api_.stopForceControl();
        api_.enableDirectTorqueMode(1.0);
        done_ = false;
    }

    virtual void write() override
    {
        AngularPosition torque;
        torque.InitStruct();

        api_.setAngularTorque(torque);

    }

    virtual void stop() override
    {
        api_.disableTorque();
        done_ = true;
    }

    virtual bool isDone() const override
    {
        return done_;
    }

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override
    {

    }


private:
    TrajectoryPoint tp_;

    std::time_t last_command_;

    bool done_;
};
#endif // GRAVITY_COMPENSATION_CONTROLLER_HPP
