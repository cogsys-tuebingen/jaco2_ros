#ifndef JACO2_STATE_H
#define JACO2_STATE_H

#include "jaco2_api.h"
#include <kinova/KinovaTypes.h>
class Jaco2State
{
public:
    Jaco2State(Jaco2API &api)
        : api_(api), readCmd_(0)
    {
        current_position_.InitStruct();
        current_velocity_.InitStruct();
        current_torque_.InitStruct();
    }

    AngularPosition getAngularPosition() const
    {
        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
        return current_position_;
    }

    AngularPosition getAngularVelocity() const
    {
        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
        return current_velocity_;
    }

    AngularPosition getAngularForce() const
    {
        std::unique_lock<std::recursive_mutex> lock(data_mutex_);
        return current_torque_;
    }

    void read()
    {

        switch (readCmd_) {
        case 0:
        {
            current_velocity_ = api_.getAngularVelocity();
            break;
        }
        case 1:
        {
            current_position_ = api_.getAngularPosition();
            break;
        }
        case 2:
        {
            current_torque_ = api_.getAngularForce();
            break;
        }
        }

        readCmd_ = (readCmd_ + 1) % 3;
    }

private:
    mutable std::recursive_mutex data_mutex_;

    Jaco2API &api_;

    int readCmd_;

    AngularPosition current_position_;
    AngularPosition current_velocity_;
    AngularPosition current_torque_;
};

#endif // JACO2_STATE_H

