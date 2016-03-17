#ifndef VELOCITYCONTROLLER_H
#define VELOCITYCONTROLLER_H
#include <ctime>
#include "jaco2_controller.h"
#include <kinova/KinovaTypes.h>
class VelocityController : public Jaco2Controller
{
public:
    VelocityController(Jaco2State &state, Jaco2API &api)
        : Jaco2Controller(state, api),
          last_command_(std::time(nullptr)),
          done_(false)
    {
        tp_.InitStruct();
        tp_.Position.Type = ANGULAR_VELOCITY;
    }

    void setVelocity(const TrajectoryPoint& tp)
    {
        tp_ = tp;
        tp_.Position.Type = ANGULAR_VELOCITY;

        last_command_ = std::time(nullptr);
        done_ = false;
    }

    virtual void write() override
    {
        if(std::time(nullptr) - last_command_ > 0.1)
        {
            tp_.InitStruct();
            tp_.Position.Type = ANGULAR_VELOCITY;
            done_ = true;
        }
        api_.setAngularVelocity(tp_);
    }

    virtual bool isDone() const override
    {
        return done_;
    }

private:
    TrajectoryPoint tp_;

    std::time_t last_command_;

    bool done_;
};
#endif // VELOCITYCONTROLLER_H
