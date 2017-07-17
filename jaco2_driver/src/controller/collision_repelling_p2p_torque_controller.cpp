#include <jaco2_driver/controller/collision_repelling_p2p_torque_controller.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <kinova/KinovaArithmetics.hpp>

CollisionReplellingP2PTorqueController::CollisionReplellingP2PTorqueController(Jaco2State &state, Jaco2API &api):
    TrajectoryTrackingController(state, api),
    reflex_controller_(state, api),
    tracking_controller_(state, api),
    collision_reaction_(state)
{
    collision_reaction_.setRobotModel("/robot_description", "jaco_link_base", "jaco_link_hand");
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();

}


void CollisionReplellingP2PTorqueController::start()
{
   tracking_controller_.start();
   api_.enableDirectTorqueMode(1.0);
}
void CollisionReplellingP2PTorqueController::stop()
{
    api_.disableTorque();
}

bool CollisionReplellingP2PTorqueController::isDone() const
{
    tracking_controller_.isDone();
}


void CollisionReplellingP2PTorqueController::setTrajectory(const JointTrajectory& trajectory)
{
    tracking_controller_.setTrajectory(trajectory);
    collision_reaction_.resetResiduals();
//    api_.disableTorque();
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();
}

void CollisionReplellingP2PTorqueController::setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg)
{
    tracking_controller_.setConfig(cfg);
    collision_reaction_.setConfig(cfg);
    reflex_controller_.setConfig(cfg);
}

void CollisionReplellingP2PTorqueController::write()
{

//    auto now = std::chrono::high_resolution_clock::now();
//    auto durationLast = now - last_cmd_rep_;
//    last_cmd_rep_ = now;
//    double dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
    collision_reaction_.update();

    double residual = collision_reaction_.getResidualsNorm();

    if(collision_reaction_.inCollision()){
        ROS_INFO_STREAM("Repelling! collision detected: "<< residual);
        auto cmd = collision_reaction_.torqueControlReflex();
        reflex_controller_.setTorque(cmd);
        reflex_controller_.write();

    }
    else{  // for now do not use energy dissipation
//        done_ =!first_coll_;
        tracking_controller_.write();
    }


}


void CollisionReplellingP2PTorqueController::setReflexGain(const AngularInfo& R)
{
    collision_reaction_.setReflexGain(R);
}

void CollisionReplellingP2PTorqueController::setCorrectionGains(const AngularInfo& kp, const AngularInfo kd)
{
    reflex_controller_.setQGains(kp.Actuator1, 0, kd.Actuator1);

}

void CollisionReplellingP2PTorqueController::setVelocityControlGains(double p, double i, double d)
{
    reflex_controller_.setGains(p, i, d);
}

void CollisionReplellingP2PTorqueController::setThreshold(double threshold)
{
    collision_reaction_.setThreshold(threshold);
}

void CollisionReplellingP2PTorqueController::setRobotModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip)
{
     collision_reaction_.setRobotModel(robot_model, chain_root, chain_tip);
}

void CollisionReplellingP2PTorqueController::setGainP(const ManipulatorInfo &gains)
{
    tracking_controller_.setGainP(gains);
}
void CollisionReplellingP2PTorqueController::setGainI(const ManipulatorInfo &gains)
{
    tracking_controller_.setGainI(gains);
}
void CollisionReplellingP2PTorqueController::setGainD(const ManipulatorInfo &gains)
{
    tracking_controller_.setGainD(gains);
}

AngularInfo CollisionReplellingP2PTorqueController::getJointError() const
{
    return tracking_controller_.getJointError();
}
