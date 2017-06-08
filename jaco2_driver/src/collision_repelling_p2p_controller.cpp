#include <jaco2_driver/collision_repelling_p2p_controller.h>
#include <jaco2_driver/jaco2_driver_constants.h>

CollisionReplellingP2PController::CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api):
    Point2PointVelocityController(state, api),
    threshold_(1.0),
    first_coll_(true),
    resiudals_("robot_description", "jaco_link_base", "jaco_link_hand"),
    estimator_("/robot_description","jaco_link_base","jaco_link_hand")
{
    setRobotModel("/robot_description", "jaco_link_base", "jaco_link_hand");
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();

    kr_.InitStruct();
    kpq_.InitStruct();
    kdq_.InitStruct();
}

void CollisionReplellingP2PController::write()
{

    auto now = std::chrono::high_resolution_clock::now();
    auto durationLast = now - last_cmd_rep_;
    last_cmd_rep_ = now;
//    double timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6;
    dt_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;


    double residual = getResiduals();
//    double residual = 0;

    if(residual > threshold_){
        // repell

        reflex();
       ROS_INFO_STREAM("Repelling! collision detected: "<< residual);
    }
    else{
        done_ =!first_coll_;
        Point2PointVelocityController::write();
    }


}

void CollisionReplellingP2PController::reflex()
{

    AngularInfo cmd;
    cmd.InitStruct();
    cmd.Actuator1 = kr_.Actuator1 * last_residual_(0);
    cmd.Actuator2 = kr_.Actuator2 * last_residual_(1);
    cmd.Actuator3 = kr_.Actuator3 * last_residual_(2);
    cmd.Actuator4 = kr_.Actuator4 * last_residual_(3);
    cmd.Actuator5 = kr_.Actuator5 * last_residual_(4);
    cmd.Actuator6 = kr_.Actuator6 * last_residual_(5);

//    auto new_torque = state_.getTorqueGFree();
    auto pos = state_.getAngularPosition();
    auto vel = state_.getAngularPosition();

    DataConversion::from_degrees(pos);
    DataConversion::from_degrees(vel);

    Jaco2KinDynLib::IntegrationData data;
    DataConversion::convert(pos.Actuators, data.pos);
    DataConversion::convert(vel.Actuators, data.vel);
    if(first_coll_){
        data.dt = 0;
        estimator_.setInitalValues(data);
        api_.enableDirectTorqueMode(1.0,0.5);
        first_coll_ = false;
    }

    DataConversion::convert(cmd, data.torques);
    data.dt = dt_;

    estimator_.estimateGfree(data);
    std::vector<double> desired_pos = estimator_.getCurrentPosition();
    std::vector <double> desired_vel = estimator_.getCurrentVelocity();
    AngularInfo diffQ = desired_pos - pos.Actuators;
    AngularInfo diffV = desired_vel - vel.Actuators;

    cmd += kpq_ * diffQ +
           kdq_ * diffV;

    api_.setAngularTorque(cmd);
    //alternative solution velocity control
//    TrajectoryPoint tp;
//    tp.InitStruct();
//    tp.Position.Type = ANGULAR_VELOCITY;
//    tp.Position.HandMode = HAND_NOMOVEMENT;
//    tp.Position.Actuators = kpq_ * diffQ + kdq_ * diffV;
//    api_.setAngularVelocity(tp);
}

void CollisionReplellingP2PController::setReflexGain(const AngularInfo& R)
{
    kr_ = R;
}

void CollisionReplellingP2PController::setCorrectionGains(const AngularInfo& kp, const AngularInfo kd)
{
    kpq_ = kp;
    kdq_ = kd;
}

bool CollisionReplellingP2PController::isDone() const
{
    return done_;
}


void CollisionReplellingP2PController::setThreshold(double threshold)
{
    threshold_ = threshold;
}

void CollisionReplellingP2PController::setRobotModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip)
{
     resiudals_ = Jaco2ResidualVector(robot_model, chain_root, chain_tip);
     estimator_.setModel(robot_model, chain_root, chain_tip);
     resiudals_.setGravity(0,0,0);
}


double CollisionReplellingP2PController::getResiduals()
{
    ResidualData data;

    getResidualsData(data);

    Eigen::VectorXd new_integral(Jaco2DriverConstants::n_Jaco2Joints);
    new_integral.setZero(Jaco2DriverConstants::n_Jaco2Joints);

    Eigen::VectorXd new_residual(Jaco2DriverConstants::n_Jaco2Joints);
    new_residual.setZero(Jaco2DriverConstants::n_Jaco2Joints);

    resiudals_.getResidualVector(data,last_residual_,last_integral_,new_integral,new_residual);

    last_integral_ = new_integral;
    last_residual_ = new_residual;

    double norm = new_residual.norm();
    return norm;
}

void CollisionReplellingP2PController::getResidualsData(ResidualData &data)
{

    auto torques = state_.getAngularForce();
    auto vel = state_.getAngularVelocity();
    auto pos = state_.getAngularPosition();
    estimateGravity(data.gx, data.gy, data.gz);
    if(filter_g_.size() == 1)
    {
        return;
    }
    data.dt = dt_;

    data.joint_positions.resize(Jaco2DriverConstants::n_Jaco2Joints);
    data.joint_velocities.resize(Jaco2DriverConstants::n_Jaco2Joints);
    data.torques.resize(Jaco2DriverConstants::n_Jaco2Joints);

    data.joint_positions[0] = pos.Actuators.Actuator1;
    data.joint_positions[1] = pos.Actuators.Actuator2;
    data.joint_positions[2] = pos.Actuators.Actuator3;
    data.joint_positions[3] = pos.Actuators.Actuator4;
    data.joint_positions[4] = pos.Actuators.Actuator5;
    data.joint_positions[5] = pos.Actuators.Actuator6;

    data.joint_velocities[0] = vel.Actuators.Actuator1;
    data.joint_velocities[1] = vel.Actuators.Actuator2;
    data.joint_velocities[2] = vel.Actuators.Actuator3;
    data.joint_velocities[3] = vel.Actuators.Actuator4;
    data.joint_velocities[4] = vel.Actuators.Actuator5;
    data.joint_velocities[5] = vel.Actuators.Actuator6;

    data.torques[0] = torques.Actuators.Actuator1;
    data.torques[1] = torques.Actuators.Actuator2;
    data.torques[2] = torques.Actuators.Actuator3;
    data.torques[3] = torques.Actuators.Actuator4;
    data.torques[4] = torques.Actuators.Actuator5;
    data.torques[5] = torques.Actuators.Actuator6;

}

void CollisionReplellingP2PController::estimateGravity(double& gx, double &gy, double& gz)
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
