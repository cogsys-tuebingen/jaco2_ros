#ifndef VELOCITYCONTROLLER_H
#define VELOCITYCONTROLLER_H
#include <ctime>
#include "jaco2_controller.h"
#include <kinova/KinovaTypes.h>
#include <kinova/KinovaArithmetics.hpp>
using namespace KinovaArithmetics;

class VelocityController : public Jaco2Controller
{
public:
    VelocityController(Jaco2State &state, Jaco2API &api)
        : Jaco2Controller(state, api),
          //          last_command_(std::time(nullptr)),
          last_command_(std::chrono::high_resolution_clock::now()),
          kp_(1.2),
          ki_(0.0),
          kd_(0.0),
          samplingPeriod_(1.0/65.0),
          counter_(0)
    {
        cmd_.InitStruct();
        cmd_.Position.Type = ANGULAR_VELOCITY;
        desired_.InitStruct();
        desired_.Position.Type = ANGULAR_VELOCITY;
    }

    virtual void start() override
    {
//        api_.disableTorque();

        last_diff_.InitStruct();
    }

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override
    {
        kp_ = cfg.velocity_controller_p_gain;
        ki_ = cfg.velocity_controller_i_gain;
        kd_ = cfg.velocity_controller_p_gain;
    }

    virtual void setVelocity(const TrajectoryPoint& tp)
    {
        desired_ = tp;
        desired_.Position.Type = ANGULAR_VELOCITY;
        desired_.Position.HandMode = HAND_NOMOVEMENT;

        last_command_ = std::chrono::high_resolution_clock::now();
        done_ = false;
        result_ = ControllerResult::WORKING;
    }

    void setGains(double p, double i, double d)
    {
        kp_ = p;
        ki_ = i;
        kd_ = d;
    }

    void setFingerPosition(const TrajectoryPoint& tp)
    {

        desired_.Position.Actuators.Actuator1 = 0;
        desired_.Position.Actuators.Actuator2 = 0;
        desired_.Position.Actuators.Actuator3 = 0;
        desired_.Position.Actuators.Actuator4 = 0;
        desired_.Position.Actuators.Actuator5 = 0;
        desired_.Position.Actuators.Actuator6 = 0;

        desired_.Position.Fingers = tp.Position.Fingers;
        desired_.Position.Type = ANGULAR_VELOCITY;
        desired_.Position.HandMode = VELOCITY_MODE;
        //        last_command_ = std::time(nullptr);
        last_command_ = std::chrono::high_resolution_clock::now();
        done_ = false;
    }

    virtual void write() override
    {
        auto now = std::chrono::high_resolution_clock::now();

        auto durationLast = now - last_command_;

        samplingPeriod_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;

        double sum = absSum(desired_.Position.Actuators);

        if(samplingPeriod_ > 0.05)
        {
            stopMotion();
            esum_.InitStruct();
            last_diff_.InitStruct();
            counter_ = 0;
            done_ = true;
            result_ = ControllerResult::SUCCESS;
            desired_.Position.InitStruct();
            desired_.Position.Type = ANGULAR_VELOCITY;
            return;
        }
        else if(desired_.Position.HandMode == HAND_NOMOVEMENT && sum > 0.01){
            auto vel = pidControl();
            cmd_.Position.Actuators = vel;
//            cmd_.Position.Actuators = desired_.Position.Actuators;
//            std::cout << "controller command vel: " << KinovaArithmetics::to_string(cmd_.Position.Actuators ) <<std::endl;

        }
//        std::cout << "desired vel: "<< desired_.Position.Actuators.Actuator6 <<std::endl;
//        std::cout << "cmd vel: "<< cmd_.Position.Actuators.Actuator6 <<std::endl;
        api_.setAngularVelocity(cmd_);

    }

    virtual bool isDone() const override
    {
        return done_;
    }

    void stopMotion()
    {
        cmd_.InitStruct();
        cmd_.Position.Type = ANGULAR_VELOCITY;
        for(int i = 0; i < 2; ++i){
            api_.setAngularVelocity(cmd_);
            usleep(5000);
        }
    }

private:
    AngularInfo pidControl()
    {
        auto new_vel = state_.getAngularVelocity();

        vel_buffer_.push_back(new_vel.Actuators);
        if(vel_buffer_.size() > 5){
            vel_buffer_.pop_front();
        }
        auto vel = meanOfVelBuffer();

        AngularInfo res;
        res.InitStruct();
        AngularInfo diff = desired_.Position.Actuators - vel;

        calculateEsum(diff);

        double kd = kd_/samplingPeriod_;
        if(std::isnan(kd)){
            kd = 0;
        }
        else if( std::abs(kd)> 1e8){
            kd = 0;
        }

        res = 1.0 * desired_.Position.Actuators
                + kp_ * diff
                + ki_ * esum_
                + kd * (diff - last_diff_) ;
        last_diff_ = diff;


        return res;
    }

    AngularInfo meanOfVelBuffer()
    {
        AngularInfo res;
        res.InitStruct();
        for(auto d : vel_buffer_){
            res += d;
        }
        res *= 1.0 / ((double) vel_buffer_.size());
        return res;
    }
    void calculateEsum(const AngularInfo& diff)
    {
        e_buffer_.push_back(diff);
        if(e_buffer_.size() > 130){
            e_buffer_.pop_front();
        }
        esum_.InitStruct();
        for(auto val : e_buffer_){
            esum_ = esum_ + samplingPeriod_ * val ;
        }
    }

protected:
    TrajectoryPoint desired_;
private:
    TrajectoryPoint cmd_;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_;

    double kp_;
    double ki_;
    double kd_;
    double samplingPeriod_;
    AngularInfo last_diff_;
    AngularInfo esum_;
    int counter_;
    std::deque<AngularInfo> vel_buffer_;
    std::deque<AngularInfo> e_buffer_;

};
#endif // VELOCITYCONTROLLER_H
