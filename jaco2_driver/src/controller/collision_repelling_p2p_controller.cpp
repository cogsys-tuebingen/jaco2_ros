#include <jaco2_driver/controller/collision_repelling_p2p_controller.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <kinova/KinovaArithmetics.hpp>

CollisionReplellingP2PController::CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api):
    Jaco2Controller(state, api),
    reflex_controller_(state, api),
    tracking_controller_(state, api),
    collision_reaction_(state)
{
    collision_reaction_.setRobotModel("/robot_description", "jaco_link_base", "jaco_link_hand");
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
//    api_.disableTorque();
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();
}


void CollisionReplellingP2PController::write()
{

    auto now = std::chrono::high_resolution_clock::now();
    auto durationLast = now - last_cmd_rep_;
    last_cmd_rep_ = now;
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
    collision_reaction_.update(dt);

    double residual = collision_reaction_.getResidualsNorm();

    if(collision_reaction_.inCollision()){
        ROS_INFO_STREAM("Repelling! collision detected: "<< residual);
        auto cmd = collision_reaction_.velocityControlReflex();
        reflex_controller_.setVelocity(cmd);
        for(int i = 0; i < 4; ++i){
            reflex_controller_.write();
            usleep(5000);
        }

    }
    else{  // for now do not use energy dissipation
//        done_ =!first_coll_;
        tracking_controller_.write();
    }


}


void CollisionReplellingP2PController::setReflexGain(const AngularInfo& R)
{
    collision_reaction_.setReflexGain(R);
}

void CollisionReplellingP2PController::setCorrectionGains(const AngularInfo& kp, const AngularInfo kd)
{
    collision_reaction_.setVelocityGains(kp, kd);

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
