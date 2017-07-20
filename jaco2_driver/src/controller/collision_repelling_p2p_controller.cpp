#include <jaco2_driver/controller/collision_repelling_p2p_controller.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <kinova/KinovaArithmetics.hpp>

CollisionReplellingP2PController::CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api):
    TrajectoryTrackingController(state, api),
    set_model_(false),
    reflex_controller_(state, api),
    tracking_controller_(state, api),
    collision_reaction_(state)
{
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();
}


void CollisionReplellingP2PController::start()
{
   tracking_controller_.start();
}

bool CollisionReplellingP2PController::isDone() const
{
    tracking_controller_.isDone();
}


void CollisionReplellingP2PController::setTrajectory(const JointTrajectory& trajectory)
{
    tracking_controller_.setTrajectory(trajectory);
    collision_reaction_.resetResiduals();
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();
    done_ = false;
    result_ = ControllerResult::WORKING;
}


void CollisionReplellingP2PController::write()
{
    if(!set_model_){
        return;
    }

//    auto now = std::chrono::high_resolution_clock::now();
//    auto durationLast = now - last_cmd_rep_;
//    last_cmd_rep_ = now;
//    double dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
    collision_reaction_.update();

    double residual = collision_reaction_.getResidualsNorm();

    if(collision_reaction_.inCollision()){
        while(collision_reaction_.inCollision()){ // Republish problem?
            ROS_WARN_STREAM("Repelling! collision detected: "<< residual);
            auto cmd = collision_reaction_.velocityControlReflex();
            reflex_controller_.setVelocity(cmd);
            reflex_controller_.write();
            usleep(5000);
            state_.read();
            collision_reaction_.update();
            residual = collision_reaction_.getResidualsNorm();
        }

        done_ = true;
        return;
    }
    else{  // for now do not use energy dissipation
        tracking_controller_.write();
        done_ = tracking_controller_.isDone();
        result_ = tracking_controller_.getResult();
    }


}

void CollisionReplellingP2PController::setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg)
{
    tracking_controller_.setConfig(cfg);
    collision_reaction_.setConfig(cfg);
    reflex_controller_.setConfig(cfg);
    set_model_ = true;
}


void CollisionReplellingP2PController::setReflexGain(const AngularInfo& R)
{
    collision_reaction_.setReflexGain(R);
}

void CollisionReplellingP2PController::setVelocityControlGains(double p, double i, double d)
{
    reflex_controller_.setGains(p, i, d);
}

void CollisionReplellingP2PController::setThreshold(double threshold)
{
    collision_reaction_.setThreshold(threshold);
}

void CollisionReplellingP2PController::setRobotModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip)
{
     collision_reaction_.setRobotModel(robot_model, chain_root, chain_tip);
     set_model_ = true;
}

void CollisionReplellingP2PController::setGainP(const ManipulatorInfo &gains)
{
    tracking_controller_.setGainP(gains);
}
void CollisionReplellingP2PController::setGainI(const ManipulatorInfo &gains)
{
    tracking_controller_.setGainI(gains);
}
void CollisionReplellingP2PController::setGainD(const ManipulatorInfo &gains)
{
    tracking_controller_.setGainD(gains);
}

AngularInfo CollisionReplellingP2PController::getJointError() const
{
    return tracking_controller_.getJointError();
}
