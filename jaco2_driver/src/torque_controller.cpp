#include <ctime>
#include <jaco2_driver/torque_controller.h>
#include <jaco2_driver/data_conversion.h>
#include <kinova/KinovaTypes.h>
#include <kinova/KinovaArithmetics.hpp>


using namespace KinovaArithmetics;

TorqueController::TorqueController(Jaco2State &state, Jaco2API &api)
    : Jaco2Controller(state, api),
      //          last_command_(std::time(nullptr)),
      estimator_("/robot_description","jaco_link_base","jaco_link_hand"),
      last_command_(std::chrono::high_resolution_clock::now()),
      kp_(1.2),
      ki_(0.0),
      kd_(0.0),
      kqp_(1.0),
      kqi_(0.0),
      kqd_(0.0),
      samplingPeriod_(1.0/65.0)
{
    cmd_.InitStruct();
    desired_.InitStruct();
    esum_.InitStruct();
    esumQ_.InitStruct();
    //See Torque Control Documentation for the Jaco 2.
    max_torques_.Actuator1 = 19.0;
    max_torques_.Actuator2 = 38.0;
    max_torques_.Actuator3 = 19.0;
    max_torques_.Actuator4 = 7.0;
    max_torques_.Actuator5 = 7.0;
    max_torques_.Actuator6 = 7.0;


}

void TorqueController::start()
{
    api_.enableDirectTorqueMode(1.0);
    last_diff_.InitStruct();
    done_ = false;
    std::cout << "start torque control" << std::endl;

    auto pos = state_.getAngularPosition();
    auto vel = state_.getAngularPosition();

    DataConversion::from_degrees(pos);
    DataConversion::from_degrees(vel);

    Jaco2KinDynLib::IntegrationData data;;
    data.dt = 0;
    DataConversion::convert(pos.Actuators, data.pos);
    DataConversion::convert(vel.Actuators, data.vel);
    estimator_.setInitalValues(data);
    data.torques = estimator_.getModelTorques();

    //    estimator_.estimateGfree(data);
    esumQ_.InitStruct();
    counter_ = 0;

}

void TorqueController::setTorque(const AngularPosition& tp)
{
    desired_ = tp;
    //        last_command_ = std::time(nullptr);
    last_command_ = std::chrono::high_resolution_clock::now();
    if(done_){
        start();
    }
    api_.enableDirectTorqueMode(1.0);

    done_ = false;
    //    std::cout << "new target." << std::endl;
}

void TorqueController::setTorque(const AngularInfo& tp)
{
    desired_.Actuators = tp;
    //        last_command_ = std::time(nullptr);
    last_command_ = std::chrono::high_resolution_clock::now();
    if(done_){
        start();
    }
    api_.enableDirectTorqueMode(1.0);
    done_ = false;
    //    std::cout << "new target." << std::endl;
}

void TorqueController::setGains(double p, double i, double d)
{
    //    api_.setActuatorPID(Actuator1,p,i,d);
    kp_ = p;
    ki_ = i;
    kd_ = d;
}

void TorqueController::setQGains(double p, double i, double d)
{
    //    api_.setActuatorPID(Actuator1,p,i,d);
    kqp_ = p;
    kqi_ = i;
    kqd_ = d;
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
        api_.disableTorque();
        //        std::cout << "torque control done: " <<  now.time_since_epoch().count() << std::endl;
    }
    else{
        auto cmd = pidControl();
        cmd_.Actuators.InitStruct();

        if(std::abs(desired_.Actuators.Actuator1) > 0.1) {
            cmd_.Actuators.Actuator1 = cmd.Actuator1;
        }
        if(std::abs(desired_.Actuators.Actuator2) > 0.1){
            cmd_.Actuators.Actuator2 = cmd.Actuator2;
        }
        if(std::abs(desired_.Actuators.Actuator3) > 0.1){
            cmd_.Actuators.Actuator3 = cmd.Actuator3;
        }
        if(std::abs(desired_.Actuators.Actuator4) > 0.1){
            cmd_.Actuators.Actuator4 = cmd.Actuator4;
        }
        if(std::abs(desired_.Actuators.Actuator5) > 0.1){
            cmd_.Actuators.Actuator5 = cmd.Actuator5;
        }
        if(std::abs(desired_.Actuators.Actuator6) > 0.1){
            cmd_.Actuators.Actuator6 = cmd.Actuator6;
        }
        if(++counter_ < 4){
            std::cout << "t-control desired: counter: "<<  counter_ << "\t" << KinovaArithmetics::to_string(desired_.Actuators)<< std::endl;
            std::cout << "t-control cmd.: counter: "<<  counter_ << "\t" << KinovaArithmetics::to_string(cmd)<< std::endl;
            std::cout << kp_ << "\t" << ki_  << "\t" << kd_ << "\t" << kqp_ << "\t" << kqi_  << "\t" << kqd_   << std::endl;
        }
        api_.setAngularTorque(cmd_);
    }


}

bool TorqueController::isDone() const
{
    return done_;
}

void TorqueController::stop()
{
    api_.disableTorque();
}

AngularInfo TorqueController::pidControl()
{
    auto new_torque = state_.getTorqueGFree();
    auto pos = state_.getAngularPosition();
    auto vel = state_.getAngularPosition();


    DataConversion::from_degrees(pos);
    DataConversion::from_degrees(vel);
    //    std::cout << "Is Conf :  " << std::endl
    //              << pos.Actuators.Actuator1 << "\t"
    //              << pos.Actuators.Actuator2 << "\t"
    //              << pos.Actuators.Actuator3 << "\t"
    //              << pos.Actuators.Actuator4 << "\t"
    //              << pos.Actuators.Actuator5 << "\t"
    //              << pos.Actuators.Actuator6 << std::endl;;

    Jaco2KinDynLib::IntegrationData data;
    DataConversion::convert(pos.Actuators, data.pos);
    DataConversion::convert(vel.Actuators, data.vel);
    DataConversion::convert(desired_.Actuators, data.torques);
    data.dt = samplingPeriod_;

    estimator_.estimateGfree(data);

    std::vector<double> desired_pos = estimator_.getCurrentPosition();
    std::vector <double> desired_vel = estimator_.getCurrentVelocity();
    //    std::cout << "Est. Desired Conf: " << std::endl;
    //    for(int i = 0; i < 6 ; ++ i) {
    //        std::cout <<  desired_pos[i] << "\t";
    //    }
    //    std::cout << std::endl;
    //    torque_buffer_.push_back(new_torque.Actuators);
    //    if(torque_buffer_.size() > 5){
    //        torque_buffer_.pop_front();
    //    }
    //    auto tau = meanOfTorqueBuffer();
    //    std::cout << "g_free:  "   << std::endl
    //              << new_torque.Actuators.Actuator1 << "\t"
    //              << new_torque.Actuators.Actuator2 << "\t"
    //              << new_torque.Actuators.Actuator3 << "\t"
    //              << new_torque.Actuators.Actuator4 << "\t"
    //              << new_torque.Actuators.Actuator5 << "\t"
    //              << new_torque.Actuators.Actuator6 << std::endl;

    AngularInfo res;
    res.InitStruct();
    AngularInfo diff = desired_.Actuators - new_torque.Actuators;

    calculateEsum(diff);
    AngularInfo diffQ = desired_pos - pos.Actuators;
    AngularInfo diffV = desired_vel - vel.Actuators;

//    std::cout << "diffQ:  " << std::endl
//              << diffQ.Actuator1 << "\t"
//              << diffQ.Actuator2 << "\t"
//              << diffQ.Actuator3 << "\t"
//              << diffQ.Actuator4 << "\t"
//              << diffQ.Actuator5 << "\t"
//              << diffQ.Actuator6 << std::endl;
//    std::cout << "diffV:  " << std::endl
//              << diffV.Actuator1 << "\t"
//              << diffV.Actuator2 << "\t"
//              << diffV.Actuator3 << "\t"
//              << diffV.Actuator4 << "\t"
//              << diffV.Actuator5 << "\t"
//              << diffV.Actuator6 << std::endl;

    esumQ_ += samplingPeriod_ * diffQ;

    double kd = kd_/samplingPeriod_;
    if(std::isnan(kd)){
        kd = 0;
    }
    else if( std::abs(kd)> 1e8){
        kd = 0;
    }

    res = 1.0 * desired_.Actuators
            + kp_ * diff
            + ki_ * esum_
            + kd * (diff - last_diff_)
            + kqp_ * (diffQ)
            + kqd_ * (diffV)
            + kqi_ * (esumQ_);

    //    std::cout << kp_ << " | " << ki_ << " | " << kd << std::endl;
    last_diff_ = diff;

    auto abs_cmd = KinovaArithmetics::abs(res);
    if(abs_cmd.Actuator1 > max_torques_.Actuator1){
        res.Actuator1 = 0.95 * max_torques_.Actuator1;
    }
    if(abs_cmd.Actuator2 > max_torques_.Actuator2){
        res.Actuator2 = 0.95 *max_torques_.Actuator2;
    }
    if(abs_cmd.Actuator3 > max_torques_.Actuator3){
        res.Actuator3 = 0.95 *max_torques_.Actuator3;
    }
    if(abs_cmd.Actuator4 > max_torques_.Actuator4){
        res.Actuator4 = 0.95 * max_torques_.Actuator4;
    }
    if(abs_cmd.Actuator5 > max_torques_.Actuator5){
        res.Actuator5 = 0.95 *max_torques_.Actuator5;
    }
    if(abs_cmd.Actuator6 > max_torques_.Actuator6){
        res.Actuator6 = 0.95 * max_torques_.Actuator6;
    }

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


