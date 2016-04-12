#ifndef ANGULAR_POSITION_CONTROLLER_H
#define ANGULAR_POSITION_CONTROLLER_H

#include <kinova/KinovaTypes.h>
#include "jaco2_controller.h"

class AngularPositionController : public Jaco2Controller
{
public:
    AngularPositionController(Jaco2State &state, Jaco2API &api)
        : Jaco2Controller(state, api),
          moveToAngularPos_(false),
          reachedAngularPos_(false),
          handMode_(false),
          counter_(0)
    {
        tp_.InitStruct();
        tp_.Position.Type = ANGULAR_POSITION;
    }

    void setPosition(const TrajectoryPoint& tp)
    {
        handMode_ = false;
        tp_ = tp;
        tp_.Position.Type = ANGULAR_POSITION;
        tp_.Position.HandMode = HAND_NOMOVEMENT;
    }

    void setFingerPosition(const TrajectoryPoint& tp)
    {
        handMode_ = true;
        tp_.Position.Actuators = state_.getAngularPosition().Actuators;
        tp_.Position.Fingers = tp.Position.Fingers;
        tp_.Position.Type = ANGULAR_POSITION;
        tp_.Position.HandMode = POSITION_MODE;
    }

    virtual void write() override
    {
        //oddly we have to re publish finger position commands?!
        if((!moveToAngularPos_ && !reachedAngularPos_) || (handMode_ && counter_ == 0) )
        {
            moveToAngularPos_ = true;
            api_.setAngularPosition(tp_);
        }
        reachedAngularPos_ = reachedAngularGoal(tp_);
        if(reachedAngularPos_)
        {
            moveToAngularPos_ = false;
        }
        counter_ = (counter_ + 1) % 10;

    }

    virtual bool isDone() const
    {
        return reachedAngularPos_;
    }

private:

    bool reachedAngularGoal(const TrajectoryPoint &goal)
    {
        double thres = 1;
        double Fthres = 15;

        auto current_position =  state_.getAngularPosition();
        if(!handMode_)
        {
            double diff1 = fabs(current_position.Actuators.Actuator1 - goal.Position.Actuators.Actuator1);
            double diff2 = fabs(current_position.Actuators.Actuator2 - goal.Position.Actuators.Actuator2);
            double diff3 = fabs(current_position.Actuators.Actuator3 - goal.Position.Actuators.Actuator3);
            double diff4 = fabs(current_position.Actuators.Actuator4 - goal.Position.Actuators.Actuator4);
            double diff5 = fabs(current_position.Actuators.Actuator5 - goal.Position.Actuators.Actuator5);
            double diff6 = fabs(current_position.Actuators.Actuator6 - goal.Position.Actuators.Actuator6);

//            double diffF1 = fabs(current_position.Fingers.Finger1 -goal.Position.Fingers.Finger1);
//            double diffF2 = fabs(current_position.Fingers.Finger2 -goal.Position.Fingers.Finger2);
//            double diffF3 = fabs(current_position.Fingers.Finger3 -goal.Position.Fingers.Finger3);

            return (diff1 < thres) && (diff2 < thres) && (diff3 < thres) && (diff4 < thres) &&
                    (diff5 < thres) && (diff6 < thres); //&& (diffF1 < thres) && (diffF2 < thres) &&
//                    (diffF3 < thres);
        }
        else
        {
            double diffF1 = fabs(current_position.Fingers.Finger1 -goal.Position.Fingers.Finger1);
            double diffF2 = fabs(current_position.Fingers.Finger2 -goal.Position.Fingers.Finger2);
            double diffF3 = fabs(current_position.Fingers.Finger3 -goal.Position.Fingers.Finger3);

            return (diffF1 < Fthres) && (diffF2 < Fthres) && (diffF3 < Fthres);
        }
    }


    bool moveToAngularPos_;
    bool reachedAngularPos_;
    bool handMode_;
    int counter_;

    TrajectoryPoint tp_;
};
#endif // ANGULAR_POSITION_CONTROLLER_H

