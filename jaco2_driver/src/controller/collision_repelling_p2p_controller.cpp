#include <jaco2_driver/controller/collision_repelling_p2p_controller.h>
#include <jaco2_driver/jaco2_driver_constants.h>
#include <kinova/KinovaArithmetics.hpp>

CollisionReplellingP2PController::CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api):
    Jaco2Controller(state, api),
    reflex_controller_(state, api),
    tracking_controller_(state, api),
    threshold_(std::sqrt(6)*2),
    first_coll_(true),
    in_collision_(false),
    resiudals_("robot_description", "jaco_link_base", "jaco_link_hand"),
    estimator_("/robot_description","jaco_link_base","jaco_link_hand")
{
    setRobotModel("/robot_description", "jaco_link_base", "jaco_link_hand");
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();

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
    resetResiduals();
    api_.disableTorque();
    last_cmd_rep_  = std::chrono::high_resolution_clock::now();
    in_collision_ = false;
    first_coll_ = true;
}


void CollisionReplellingP2PController::write()
{

    auto now = std::chrono::high_resolution_clock::now();
    auto durationLast = now - last_cmd_rep_;
    last_cmd_rep_ = now;
    dt_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;

    double residual = getResiduals();
    in_collision_ = residual > threshold_;
    if(in_collision_){
        // repell
        ROS_INFO_STREAM("Repelling! collision detected: "<< residual);
        reflex();
        reflex_controller_.write();

    }
    else{
//        done_ =!first_coll_;
        tracking_controller_.write();
    }


}

void CollisionReplellingP2PController::reflex()
{

    AngularInfo cmd;
    cmd.Actuator1 = kr_.Actuator1 * last_residual_(0);
    cmd.Actuator2 = kr_.Actuator2 * last_residual_(1);
    cmd.Actuator3 = kr_.Actuator3 * last_residual_(2);
    cmd.Actuator4 = kr_.Actuator4 * last_residual_(3);
    cmd.Actuator5 = kr_.Actuator5 * last_residual_(4);
    cmd.Actuator6 = kr_.Actuator6 * last_residual_(5);
//    std::cout << "residual " << last_residual_ << std::endl;


    auto vel = state_.getAngularPosition();
    auto pos = state_.getAngularPosition();

    DataConversion::from_degrees(pos);
    DataConversion::from_degrees(vel);

    Jaco2KinDynLib::IntegrationData data;
    DataConversion::convert(pos.Actuators, data.pos);
    DataConversion::convert(vel.Actuators, data.vel);
    if(first_coll_){
        data.dt = 0;
        estimator_.setInitalValues(data);
        ROS_WARN_STREAM("activate torque control");
        first_coll_ = false;
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
    tp.Position.Actuators = -1.0 * kpq_ * diffQ + -1.0 * kdq_ * diffV;
    DataConversion::to_degrees(tp.Position.Actuators);
    std::cout << "vel cmd: " << KinovaArithmetics::to_string(tp.Position.Actuators ) <<std::endl;
    reflex_controller_.setVelocity(tp);
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

void CollisionReplellingP2PController::setVelocityControlGains(double p, double i, double d)
{
    reflex_controller_.setGains(p, i, d);
}

void CollisionReplellingP2PController::setThreshold(double threshold)
{
    threshold_ = threshold;
}

void CollisionReplellingP2PController::setRobotModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip)
{
     resiudals_ = Jaco2ResidualVector(robot_model, chain_root, chain_tip);
     estimator_.setModel(robot_model, chain_root, chain_tip);


     std::vector<double> r_gains(resiudals_.getNrOfJoints(), 10);
     resiudals_.setGains(r_gains);
     resetResiduals();

}

void CollisionReplellingP2PController::resetResiduals()
{
    last_integral_ = Eigen::VectorXd::Zero(resiudals_.getNrOfJoints());
    last_residual_ = Eigen::VectorXd::Zero(resiudals_.getNrOfJoints());
}

double CollisionReplellingP2PController::getResiduals()
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


    double norm = new_residual.norm();
    return norm;
}

void CollisionReplellingP2PController::getResidualsData(ResidualData &data)
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
