#include <ctime>
#include <jaco2_driver/torque_controller.h>
#include <kinova/KinovaTypes.h>
#include <kinova/KinovaArithmetics.hpp>
using namespace KinovaArithmetics;

TorqueController::TorqueController(Jaco2State &state, Jaco2API &api)
    : Jaco2Controller(state, api),
      //          last_command_(std::time(nullptr)),
      last_command_(std::chrono::high_resolution_clock::now()),
      done_(false),
      kp_(1.2),
      ki_(0.0),
      kd_(0.0),
      samplingPeriod_(1.0/65.0)
{
    cmd_.InitStruct();
    desired_.InitStruct();
}

void TorqueController::start()
{
    api_.enableDirectTorqueMode(1.0);
    last_diff_.InitStruct();
    done_ = false;
    std::cout << "start torque control" << std::endl;
}

void TorqueController::setTorque(const AngularPosition& tp)
{
    desired_ = tp;
    //        last_command_ = std::time(nullptr);
    last_command_ = std::chrono::high_resolution_clock::now();
    if(done_){
        start();
    }

    done_ = false;
    std::cout << "new target." << std::endl;
}

void TorqueController::setGains(double p, double i, double d)
{
    kp_ = p;
    ki_ = i;
    kd_ = d;
}

void TorqueController::write()
{
    auto now = std::chrono::high_resolution_clock::now();

    auto durationLast = now - last_command_;

    samplingPeriod_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;

//    std ::cout << "torque control dt: "<< samplingPeriod_ << std::endl;
    if(samplingPeriod_ > 0.05)
    {
        cmd_.InitStruct();
        esum_.InitStruct();
        last_diff_.InitStruct();

        done_ = true;
        desired_.InitStruct();
//        api_.disableTorque();
//        std::cout << "torque control done: " <<  now.time_since_epoch().count() << std::endl;
    }
    else{
        auto cmd = pidControl();
        cmd_.Actuators = cmd;
        std::cout << "cmd: joint 1: " << cmd_.Actuators.Actuator1 << std::endl;
        std::cout << "cmd: joint 2: " << cmd_.Actuators.Actuator2 << std::endl;
        std::cout << "cmd: joint 3: " << cmd_.Actuators.Actuator3 << std::endl;
        std::cout << "cmd: joint 4: " << cmd_.Actuators.Actuator4 << std::endl;
        std::cout << "cmd: joint 5: " << cmd_.Actuators.Actuator5 << std::endl;
        std::cout << "cmd: joint 6: " << cmd_.Actuators.Actuator6 << std::endl;
        api_.setAngularTorque(cmd_);
    }


}

bool TorqueController::isDone() const
{
    return done_;
}

AngularInfo TorqueController::pidControl()
{
    auto new_vel = api_.getAngularForceGravityFree();

    torque_buffer_.push_back(new_vel.Actuators);
    if(torque_buffer_.size() > 5){
        torque_buffer_.pop_front();
    }
    auto tau = meanOfTorqueBuffer();
    std::cout << "g_free: joint 1: " << tau.Actuator1 << std::endl;
    std::cout << "g_free: joint 2: " << tau.Actuator2 << std::endl;
    std::cout << "g_free: joint 3: " << tau.Actuator3 << std::endl;
    std::cout << "g_free: joint 4: " << tau.Actuator4 << std::endl;
    std::cout << "g_free: joint 5: " << tau.Actuator5 << std::endl;
    std::cout << "g_free: joint 6: " << tau.Actuator6 << std::endl;

    AngularInfo res;
    res.InitStruct();
    AngularInfo diff = desired_.Actuators - tau;

    calculateEsum(diff);

    double kd = kd_/samplingPeriod_;
    if(std::isnan(kd)){
        kd = 0;
    }
    else if( std::abs(kd)> 1e8){
        kd = 0;
    }

    res = 0.0 * desired_.Actuators
            + kp_ * diff
            + ki_ * esum_
            + kd * (diff - last_diff_) ;

    std::cout << kp_ << " | " << ki_ << " | " << kd << std::endl;
    last_diff_ = diff;



    return res;
}

AngularInfo TorqueController::meanOfTorqueBuffer() const
{
    AngularInfo res;
    res.InitStruct();
    for(auto d : torque_buffer_){
        res += d;
    }
    res *= 1.0 / ((double) torque_buffer_.size());
    return res;
}

void TorqueController::calculateEsum(const AngularInfo& diff)
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


