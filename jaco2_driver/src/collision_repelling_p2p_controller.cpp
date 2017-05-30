#include <jaco2_driver/collision_repelling_p2p_controller.h>
#include <jaco2_driver/jaco2_driver_constants.h>

CollisionReplellingP2PController::CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api):
    Point2PointVelocityController(state, api),
    resiudals_("robot_description", "jaco_link_base", "jaco_link_hand")
{
    fac_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    threshold_ = 1.0;
    setRobotModel("/robot_description", "jaco_link_base", "jaco_link_hand");
}

void CollisionReplellingP2PController::write()
{

    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - start_command_ ;
    auto durationLast = now - last_command_;
    last_command_ = now;
//    double timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6;
    samplingPeriod_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;


//    double residual = getResiduals();
    double residual = 0;

    if(residual > threshold_){
        // repell
       AngularPosition cmd;
       cmd.InitStruct();
       cmd.Actuators.Actuator1 = fac_[0] * last_residual_(0);
       cmd.Actuators.Actuator2 = fac_[1] * last_residual_(1);
       cmd.Actuators.Actuator3 = fac_[2] * last_residual_(2);
       cmd.Actuators.Actuator4 = fac_[3] * last_residual_(3);
       cmd.Actuators.Actuator5 = fac_[4] * last_residual_(4);
       cmd.Actuators.Actuator6 = fac_[5] * last_residual_(5);
       api_.enableDirectTorqueMode(1.0, 0.5);
       api_.setAngularTorque(cmd);
       ROS_INFO_STREAM("Repelling! collision detected: "<< residual);
    }
    else{
        Point2PointVelocityController::write();
    }

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
    data.dt = samplingPeriod_;

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
