#ifndef GRAVITY_COMPENSATION_CONTROLLER_HPP
#define GRAVITY_COMPENSATION_CONTROLLER_HPP
#include <ctime>
#include "jaco2_controller.h"
#include <kinova/KinovaTypes.h>
class GravityCompensationController : public Jaco2Controller
{
public:
    GravityCompensationController(Jaco2State &state, Jaco2API &api, TerminationCallback& t )
        : Jaco2Controller(state, api, t),
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
        result_ = Result::WORKING;
    }

    virtual void write() override
    {
        AngularPosition torque;
        torque.InitStruct();

        api_.setAngularTorque(torque);

    }

    void finishService()
    {
        done_ = true;
        result_ = Result::SUCCESS;
        t_(result_);
    }

    virtual void stop() override
    {
        api_.disableTorque();
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
