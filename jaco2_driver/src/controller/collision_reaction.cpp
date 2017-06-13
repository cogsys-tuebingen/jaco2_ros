#include <jaco2_driver/controller/collision_reaction.h>
#include <jaco2_driver/data_conversion.h>
#include <kinova/KinovaArithmetics.hpp>

using namespace KinovaArithmetics;

CollisionReaction::CollisionReaction(Jaco2State &state):
    state_(state),
    threshold_(std::sqrt(6)*2),
    stop_threshold_(0),
    in_collision_(false),
    collision_counter_(0),
    resiudals_("robot_description", "jaco_link_base", "jaco_link_hand"),
    estimator_("/robot_description","jaco_link_base","jaco_link_hand")
{
    kr_.InitStruct();
    kr_.Actuator1 = 1.0;
    kr_.Actuator2 = 1.0;
    kr_.Actuator3 = 1.0;
    kr_.Actuator4 = 1.0;
    kr_.Actuator5 = 1.0;
    kr_.Actuator6 = 0.0;
    kpq_.InitStruct();
    kdq_.InitStruct();
    //    esum_.InitStruct();

    for(int i = 0; i < 30; ++i)
    {
        double gx, gy, gz;
        estimateGravity(gx, gy, gz);
    }

    max_torques_.Actuator1 = 19.0;
    max_torques_.Actuator2 = 38.0;
    max_torques_.Actuator3 = 19.0;
    max_torques_.Actuator4 = 7.0;
    max_torques_.Actuator5 = 7.0;
    max_torques_.Actuator6 = 7.0;

}

void CollisionReaction::setThreshold(double threshold)
{
    threshold_ = threshold;
}

void CollisionReaction::setReflexGain(const AngularInfo &kr)
{
    kr_ = kr;
}

void CollisionReaction::setRobotModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip)
{
    resiudals_ = Jaco2ResidualVector(robot_model, chain_root, chain_tip);
    estimator_.setModel(robot_model, chain_root, chain_tip);


    std::vector<double> r_gains(resiudals_.getNrOfJoints(), 10);
    resiudals_.setGains(r_gains);
    resetResiduals();

}

void CollisionReaction::update(double dt)
{
    dt_ = dt;

    updateResiduals();

    in_collision_ = residualNorm_ > threshold_;

    if(in_collision_){
        ++collision_counter_;
    }
    else if(collision_counter_ > 0){
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


        cmd.Actuator1 = kr_.Actuator1 * last_residual_(0);
        cmd.Actuator2 = kr_.Actuator2 * last_residual_(1);
        cmd.Actuator3 = kr_.Actuator3 * last_residual_(2);
        cmd.Actuator4 = kr_.Actuator4 * last_residual_(3);
        cmd.Actuator5 = kr_.Actuator5 * last_residual_(4);
        cmd.Actuator6 = kr_.Actuator6 * last_residual_(5);

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
    ResidualData data;


    getResidualsData(data);

    Eigen::VectorXd new_integral(Jaco2DriverConstants::n_Jaco2Joints);
    new_integral.setZero(Jaco2DriverConstants::n_Jaco2Joints);

    Eigen::VectorXd new_residual(Jaco2DriverConstants::n_Jaco2Joints);
    new_residual.setZero(Jaco2DriverConstants::n_Jaco2Joints);

    resiudals_.setGravity(data.gx, data.gy, data.gz);
    resiudals_.getResidualVector(data, last_residual_, last_integral_, new_integral, new_residual);

    last_integral_ = new_integral;
    last_residual_ = new_residual;


    residualNorm_= new_residual.norm();
}

void CollisionReaction::getResidualsData(ResidualData &data)
{

    auto torques = state_.getAngularForce();
    auto vel = state_.getAngularVelocity();
    auto pos = state_.getAngularPosition();

    DataConversion::from_degrees(pos);
    DataConversion::from_degrees(vel);

    estimateGravity(data.gx, data.gy, data.gz);

    if(filter_g_.size() == 1)
    {
        return;
    }
    data.dt = dt_;

    DataConversion::convert(pos.Actuators, data.joint_positions);
    DataConversion::convert(vel.Actuators, data.joint_velocities);
    DataConversion::convert(torques.Actuators, data.torques);

}

void CollisionReaction::resetResiduals()
{
    last_integral_ = Eigen::VectorXd::Zero(resiudals_.getNrOfJoints());
    last_residual_ = Eigen::VectorXd::Zero(resiudals_.getNrOfJoints());
    collision_counter_ = 0;
}

void CollisionReaction::estimateGravity(double& gx, double &gy, double& gz)
{
    auto accs = state_.getLinearAcceleration();
    Eigen::Vector3d g_vec(accs.Actuator1_X, accs.Actuator1_Y, accs.Actuator1_Z);

    filter_g_.emplace_back(g_vec);

    while(filter_g_.size() > 30){
        filter_g_.pop_front();
    }
    Eigen::Vector3d res;
    res.setZero(3);
    for(auto v : filter_g_){
        res += v;
    }
    res *= -9.81 / filter_g_.size();

    gx = res(0);
    gy = res(1);
    gz = res(2);

}

void CollisionReaction::setVelocityGains(const AngularInfo& kp, const AngularInfo kd)
{
    kpq_ = kp;
    kdq_ = kd;
    KinovaArithmetics::invert(kpq_);
    KinovaArithmetics::invert(kdq_);

}

TrajectoryPoint CollisionReaction::calculateVelocity(AngularInfo& cmd)
{
    auto vel = state_.getAngularPosition();
    auto pos = state_.getAngularPosition();

    DataConversion::from_degrees(pos);
    DataConversion::from_degrees(vel);

    Jaco2KinDynLib::IntegrationData data;
    DataConversion::convert(pos.Actuators, data.pos);
    DataConversion::convert(vel.Actuators, data.vel);
    if(in_collision_ && collision_counter_ == 1){
        data.dt = 0;
        estimator_.setInitalValues(data);
        ROS_WARN_STREAM("activate torque control");
    }

    DataConversion::convert(cmd, data.torques);
    data.dt = dt_;

    estimator_.estimateGfree(data);
    std::vector<double> desired_pos = estimator_.getCurrentPosition();
    std::vector <double> desired_vel = estimator_.getCurrentVelocity();
    //    std::cout << "Desired Pos: ";
    //    for(auto d : desired_pos){
    //        std::cout << d << "\t";
    //    }
    //    std::cout << std::endl;
    AngularInfo diffQ = desired_pos - pos.Actuators;
    AngularInfo diffV = desired_vel - vel.Actuators;

    TrajectoryPoint tp;
    tp.InitStruct();
    tp.Position.Type = ANGULAR_VELOCITY;
    tp.Position.HandMode = HAND_NOMOVEMENT;
    tp.Position.Actuators = kpq_ * diffQ + kdq_ * diffV;
    DataConversion::to_degrees(tp.Position.Actuators);
    std::cout << "vel cmd: " << KinovaArithmetics::to_string(tp.Position.Actuators ) <<std::endl;
    return tp;
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
