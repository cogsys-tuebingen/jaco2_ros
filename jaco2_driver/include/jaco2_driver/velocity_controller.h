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
        tp_.Position.HandMode = HAND_NOMOVEMENT;

        last_command_ = std::time(nullptr);
        done_ = false;
    }

    void setFingerPosition(const TrajectoryPoint& tp)
    {

        tp_.Position.Actuators.Actuator1 = 0;
        tp_.Position.Actuators.Actuator2 = 0;
        tp_.Position.Actuators.Actuator3 = 0;
        tp_.Position.Actuators.Actuator4 = 0;
        tp_.Position.Actuators.Actuator5 = 0;
        tp_.Position.Actuators.Actuator6 = 0;

        tp_.Position.Fingers = tp.Position.Fingers;
        tp_.Position.Type = ANGULAR_VELOCITY;
        tp_.Position.HandMode = VELOCITY_MODE;
//        std::cout << tp_.Position .Fingers.Finger1 << ", " << tp_.Position.Fingers.Finger2 << ", " << tp_.Position.Fingers.Finger3 << std::endl;
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
