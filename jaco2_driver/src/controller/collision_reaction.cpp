#include <jaco2_driver/controller/collision_reaction.h>
#include <jaco2_driver/data_conversion.h>
#include <kinova/KinovaArithmetics.hpp>
#include <iostream>
#include <jaco2_data/dynamic_calibration_io.h>
#include <jaco2_data/dynamic_calibrated_parameters.hpp>
using namespace KinovaArithmetics;

CollisionReaction::CollisionReaction(Jaco2State &state):
    state_(state),
    in_collision_(false),
    initial_(true),
    collision_counter_(0),
    threshold_(std::sqrt(6)*2),
    stop_threshold_(0),
    //    dt_(0),
    residualNorm_(0)
{
    kr_.InitStruct();
    kr_.Actuator1 = 1.0;
    kr_.Actuator2 = 1.0;
    kr_.Actuator3 = 1.0;
    kr_.Actuator4 = 1.0;
    kr_.Actuator5 = 1.0;
    kr_.Actuator6 = 0.0;

    max_torques_.Actuator1 = 19.0;
    max_torques_.Actuator2 = 38.0;
    max_torques_.Actuator3 = 19.0;
    max_torques_.Actuator4 = 7.0;
    max_torques_.Actuator5 = 7.0;
    max_torques_.Actuator6 = 7.0;

}

void CollisionReaction::start()
{
    in_collision_ = false;
    collision_counter_ = 0;
    resetResiduals();
}

void CollisionReaction::setThreshold(double threshold)
{
    threshold_ = threshold;
}

void CollisionReaction::setReflexGain(const AngularInfo &kr)
{
    kr_ = kr;
    std::cout << "kr:\t" << KinovaArithmetics::to_string(kr_) <<std::endl;
}

void CollisionReaction::setRobotModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip)
{
    std::cout << robot_model<< " "<< chain_root << " " <<chain_tip<< std::endl;
    ROS_INFO_STREAM(robot_model<< " "<< chain_root << " " <<chain_tip);
    resiudals_ = Jaco2KinDynLib::Jaco2ResidualVector(robot_model, chain_root, chain_tip);
    n_joints_ = resiudals_.getNrOfJoints();
    ROS_INFO_STREAM("#joints: " << n_joints_);
    std::vector<double> r_gains(n_joints_, 10);
    resiudals_.setGains(r_gains);
    resetResiduals();
    bias_.setZero(n_joints_);
    setDynModelCalibration();
}

void CollisionReaction::update()
{

    updateResiduals();

    in_collision_ = residualNorm_ > threshold_;

    if(in_collision_){
        ++collision_counter_;
    }
    else if(collision_counter_ > 0){
        bias_ = last_residual_;
        collision_counter_ = 0;
    }
}

TrajectoryPoint CollisionReaction::velocityControlReflex()
{

    AngularInfo cmd = torqueControlReflex();
    return calculateVelocity(cmd);

}

TrajectoryPoint CollisionReaction::velocityEnergyDisspation()
{
    AngularInfo cmd = torqueControlEnergyDisspation();
    return calculateVelocity(cmd);
}


AngularInfo CollisionReaction::torqueControlReflex()
{
    AngularInfo cmd;
    cmd.InitStruct();

    Eigen::VectorXd unbiased = last_residual_ - bias_;

    //    cmd.Actuator1 = kr_.Actuator1 * last_residual_(0);
    //    cmd.Actuator2 = kr_.Actuator2 * last_residual_(1);
    //    cmd.Actuator3 = kr_.Actuator3 * last_residual_(2);
    //    cmd.Actuator4 = kr_.Actuator4 * last_residual_(3);
    //    cmd.Actuator5 = kr_.Actuator5 * last_residual_(4);
    //    cmd.Actuator6 = kr_.Actuator6 * last_residual_(5);

    cmd.Actuator1 = kr_.Actuator1 * unbiased(0);
    cmd.Actuator2 = kr_.Actuator2 * unbiased(1);
    cmd.Actuator3 = kr_.Actuator3 * unbiased(2);
    cmd.Actuator4 = kr_.Actuator4 * unbiased(3);
    cmd.Actuator5 = kr_.Actuator5 * unbiased(4);
    cmd.Actuator6 = kr_.Actuator6 * unbiased(5);

    return cmd;
}

AngularInfo CollisionReaction::torqueControlEnergyDisspation()
{
    AngularInfo cmd;
    std::vector<double> g = resiudals_.getGravityTorque();
    AngularInfo taum = -1.0 * (max_torques_ + g);
    AngularInfo tauM = (max_torques_ - g);
    auto vel = state_.getAngularVelocity();
    cmd.Actuator1 = energyDisipation(vel.Actuators,taum,tauM,1);
    cmd.Actuator2 = energyDisipation(vel.Actuators,taum,tauM,2);
    cmd.Actuator3 = energyDisipation(vel.Actuators,taum,tauM,3);
    cmd.Actuator4 = energyDisipation(vel.Actuators,taum,tauM,4);
    cmd.Actuator5 = energyDisipation(vel.Actuators,taum,tauM,5);
    cmd.Actuator6 = energyDisipation(vel.Actuators,taum,tauM,6);

    return cmd;
}

AngularInfo CollisionReaction::getResiduals() const
{
    AngularInfo result;
    result.Actuator1 = last_residual_(0);
    result.Actuator2 = last_residual_(1);
    result.Actuator3 = last_residual_(2);
    result.Actuator4 = last_residual_(3);
    result.Actuator5 = last_residual_(4);
    result.Actuator6 = last_residual_(5);
    return result;
}

double CollisionReaction::getResidualsNorm() const
{
    return residualNorm_;
}

bool CollisionReaction::inCollision() const
{
    return in_collision_;
}

bool CollisionReaction::energyDisipation() const
{
    auto vel = state_.getAngularVelocity();
    double v = KinovaArithmetics::norm(vel.Actuators);
    return residualNorm_ <= stop_threshold_ || (v > 0.1 && !inCollision());
}

bool CollisionReaction::firstCollision() const
{
    return collision_counter_ > 0;
}

std::size_t CollisionReaction::getCollisionCounter() const
{
    return collision_counter_;
}

void CollisionReaction::updateResiduals()
{
    if(robot_model_ == "" || base_link_ == "" || tip_link_ == ""){
       ROS_ERROR_STREAM("Robot will not perform any movement. Robot model not given!");
       return;
    }

    Jaco2KinDynLib::ResidualData data;

    auto state_stamped = state_.getJointState();
    auto state = state_stamped.data;

    data.gx = state.gravity(0);
    data.gy = state.gravity(1);
    data.gz = state.gravity(2);


    data.joint_positions.insert(data.joint_positions.begin(), state.position.begin(), state.position.begin() + n_joints_);
    data.joint_velocities.insert(data.joint_velocities.begin(), state.velocity.begin(), state.velocity.begin() + n_joints_);
    data.torques.insert(data.torques.begin(), state.torque.begin(), state.torque.begin() + n_joints_);

    //    data.dt = dt_;
    if(initial_){
        data.dt = 1./65;
        initial_ = false;
    }
    else{
        data.dt = std::abs(jaco2_data::TimeStamp::timeDiffinSeconds(state_stamped.stamp(), last_state_.stamp()));
    }
    //    std::cout << "collision rreaction: dt = " << data.dt <<" | " << jaco2_data::TimeStamp::timeDiffinSeconds(state.stamp, last_state_.stamp) << std::endl;
    last_state_ = state_stamped;

    if(data.dt == 0 ){
        return;
    }


    Eigen::VectorXd new_integral(Jaco2DriverConstants::n_Jaco2Joints);
    new_integral.setZero(Jaco2DriverConstants::n_Jaco2Joints);

    Eigen::VectorXd new_residual(Jaco2DriverConstants::n_Jaco2Joints);
    new_residual.setZero(Jaco2DriverConstants::n_Jaco2Joints);

    //    resiudals_.setGravity(data.gx, data.gy, data.gz);
    resiudals_.getResidualVector(data, last_residual_, last_integral_, new_integral, new_residual);

    last_integral_ = new_integral;
    last_residual_ = new_residual;

    //    std::cout << "resiudals "<< last_residual_ << std::endl;
    residualNorm_= new_residual.norm();
}


void CollisionReaction::resetResiduals()
{
    last_state_ = jaco2_data::JointStateData(Jaco2DriverConstants::n_Jaco2Joints);
    last_integral_ = Eigen::VectorXd(n_joints_);
    last_integral_.setZero(n_joints_);

    last_residual_ = Eigen::VectorXd(n_joints_);
    last_residual_.setZero(n_joints_);
    collision_counter_ = 0;
    initial_ = true;
    ROS_INFO("Rest Residuals");
}


TrajectoryPoint CollisionReaction::calculateVelocity(AngularInfo& cmd)
{
    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Type = ANGULAR_VELOCITY;
    tp.Position.HandMode = HAND_NOMOVEMENT;
    //    tp.Position.Actuators = kpq_ * diffQ + kdq_ * diffV;
    tp.Position.Actuators = cmd;
    DataConversion::to_degrees(tp.Position.Actuators);
    std::cout << "cmd:\t" << KinovaArithmetics::to_string(cmd) <<std::endl;
    std::cout << "vel cmd:\t" << KinovaArithmetics::to_string(tp.Position.Actuators ) <<std::endl;
    std::cout << "residual:\t"
              << last_residual_(0) << "\t"
              << last_residual_(1) << "\t"
              << last_residual_(2) << "\t"
              << last_residual_(3) << "\t"
              << last_residual_(4) << "\t"
              << last_residual_(5) << "\t" << std::endl;
    return tp;
}

void CollisionReaction::setConfig(jaco2_driver::jaco2_driver_configureConfig &cfg)
{
    kr_.Actuator1 = cfg.collision_reflex_gain_joint_0;
    kr_.Actuator2 = cfg.collision_reflex_gain_joint_1;
    kr_.Actuator3 = cfg.collision_reflex_gain_joint_2;
    kr_.Actuator4 = cfg.collision_reflex_gain_joint_3;
    kr_.Actuator5 = cfg.collision_reflex_gain_joint_4;
    kr_.Actuator6 = cfg.collision_reflex_gain_joint_5;

    velocity_threshold_.Actuator1 = cfg.collision_ed_vel_eps_joint_0;
    velocity_threshold_.Actuator2 = cfg.collision_ed_vel_eps_joint_1;
    velocity_threshold_.Actuator3 = cfg.collision_ed_vel_eps_joint_2;
    velocity_threshold_.Actuator4 = cfg.collision_ed_vel_eps_joint_3;
    velocity_threshold_.Actuator5 = cfg.collision_ed_vel_eps_joint_4;
    velocity_threshold_.Actuator6 = cfg.collision_ed_vel_eps_joint_5;

    threshold_ = cfg.collision_threshold;
    stop_threshold_ = cfg.collision_stop_threshold;
    std::string rmodel = cfg.robot_model_param_sever;
    std::string base = cfg.robot_model_base_link;
    std::string tip = cfg.robot_model_tip_link;
    if(rmodel != robot_model_ || base != base_link_ || tip != tip_link_){
        robot_model_ = rmodel;
        base_link_ = base;
        tip_link_ = tip;
        setRobotModel(robot_model_,base_link_, tip_link_);
    }

    loadDynModelCalibration(cfg.dynamic_model_calibration_file);


}

double CollisionReaction::energyDisipation(AngularInfo& velocity, AngularInfo& lower_lim, AngularInfo& upper_lim, std::size_t id) const
{
    double vel,lower,upper,threshold;
    switch (id) {
    case 1:{
        vel = velocity.Actuator1;
        upper = upper_lim.Actuator1;
        lower = lower_lim.Actuator1;
        threshold = velocity_threshold_.Actuator1;
        break;
    }
    case 2:{
        vel = velocity.Actuator2;
        upper = upper_lim.Actuator2;
        lower = lower_lim.Actuator2;
        threshold = velocity_threshold_.Actuator2;
        break;
    }
    case 3:{
        vel = velocity.Actuator3;
        upper = upper_lim.Actuator3;
        lower = lower_lim.Actuator3;
        threshold = velocity_threshold_.Actuator3;
        break;
    }
    case 4:{
        vel = velocity.Actuator4;
        upper = upper_lim.Actuator4;
        lower = lower_lim.Actuator4;
        threshold = velocity_threshold_.Actuator4;
        break;
    }
    case 5:{
        vel = velocity.Actuator5;
        upper = upper_lim.Actuator5;
        lower = lower_lim.Actuator5;
        threshold = velocity_threshold_.Actuator5;
        break;
    }
    case 6:{
        vel = velocity.Actuator6;
        upper = upper_lim.Actuator6;
        lower = lower_lim.Actuator6;
        threshold = velocity_threshold_.Actuator6;
        break;
    }
    default:
        break;
    }

    if(vel >= threshold){
        return lower;
    }
    if(threshold > vel >= 0){
        return lower*vel/threshold;
    }
    if(-threshold < vel  <= 0){
        return upper*vel/threshold;
    }
    if(vel <= - threshold){
        return upper;
    }
}

void CollisionReaction::loadDynModelCalibration(std::string file_name)
{
    if( file_name != ""){
        model_calib_.clear();
        Jaco2Calibration::DynCalibrationIO::loadDynParm(file_name, model_calib_);
    }
}

void CollisionReaction::setDynModelCalibration()
{
    for(const Jaco2Calibration::DynamicParameters& p : model_calib_){
        resiudals_.changeDynamicParams(p.linkName, p.mass, p.coM, p.inertia);
    }
}
