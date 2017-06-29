#include <jaco2_driver/controller/torque_trajectory_controller.h>
#include <kinova/KinovaArithmetics.hpp>

TorqueTrajectoryController::TorqueTrajectoryController(Jaco2State &state, Jaco2API &api):
    TrajectoryTrackingController(state, api),
    model_("/robot_description","jaco_link_base","jaco_link_hand"),
    nh("~")
{
    pub = nh.advertise<sensor_msgs::JointState>("/torque_control_cmd",2);
    pub2 = nh.advertise<sensor_msgs::JointState>("/torque_control_cmd2",2);
}

void TorqueTrajectoryController::setTrajectory(const JointTrajectory& trajectory)
{
    api_.enableDirectTorqueMode(1.0,0.5);
    trajectoryWrapper_.setTrajectory(trajectory);
}


void TorqueTrajectoryController::start()
{
    api_.enableDirectTorqueMode(1.0,0.5);

    done_ = false;
}

void TorqueTrajectoryController::stop()
{
    api_.disableTorque();
}

void TorqueTrajectoryController::write()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - start_command_ ;
    auto durationLast = now -last_command_;
    last_command_ = now;
    double timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6;
    trajectoryWrapper_.samplingPeriod_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;

    while(timeDiff > trajectoryWrapper_.trajectory_.getTimeFromStart(current_point_))
    {
        ++current_point_;
        trajectoryWrapper_.current_point_ = current_point_;
        if(current_point_ >= trajectoryWrapper_.trajectory_.size() )
        {
            std::cout << "done " << std::endl;
            done_ = true;
            // publish zero velocity
            cmd_.InitStruct();

            //            api_.disableTorque();
            trajectoryWrapper_.evaluationOutput();
            //            api_.setAngularTorque(tp_.Position.Actuators);
            api_.disableTorque();
            return;
        }
    }
    if(fabs(timeDiff -  trajectoryWrapper_.trajectory_.getTimeFromStart(current_point_)) < 0.02)
    {
        AngularInfo conf = state_.getAngularPosition().Actuators;
        trajectoryWrapper_.setPositionDifference(conf);
    }
    if(current_point_ <= 0)
    {
        return;
    }
    double dt = timeDiff - trajectoryWrapper_.trajectory_.getTimeFromStart(current_point_-1);
    if(current_point_ < trajectoryWrapper_.trajectory_.size())
    {
        //        std::cout << "case 0 " << current_point_  << " dt = " << dt  << " | timeDiff = " << timeDiff
        //                  << " | traj dur = " << timeDiff_[current_point_] << std::endl;
        control(dt);

        //        std::cout << "cmd torque :  "
        //                  << tp_.Position.Actuators.Actuator1 << "\t"
        //                  << tp_.Position.Actuators.Actuator2 << "\t"
        //                  << tp_.Position.Actuators.Actuator3 << "\t"
        //                  << tp_.Position.Actuators.Actuator4 << "\t"
        //                  << tp_.Position.Actuators.Actuator5 << "\t"
        //                  << tp_.Position.Actuators.Actuator6 << std::endl;;
        api_.setAngularTorque(cmd_);

    }

    pub.publish(joint_state);
    pub2.publish(joint_state2);


}
bool TorqueTrajectoryController::isDone() const
{
    return done_;
}

void TorqueTrajectoryController::control(const double dt)
{
    //The Jaco2 uses commands in degrees!
    AngularPosition currentPos = state_.getAngularPosition();
    AngularPosition currentVel = state_.getAngularVelocity();
    ManipulatorInfo diff;
    ManipulatorInfo d_diff;

    std::vector<double> desiredPos, desiredVel, desiredAcc, desiredTorques;
    getDesiredPosition(dt, desiredPos);
    getDesiredVelocity(dt, desiredVel);
    getDesiredAcceleration(dt, desiredAcc);

    diff.getAngularInfo() = desiredPos - currentPos.Actuators;
    diff.normalizeAngleDegrees();
    d_diff.getAngularInfo() = desiredVel - currentVel.Actuators;
    trajectoryWrapper_.eSum_ = diff * trajectoryWrapper_.samplingPeriod_;
    trajectoryWrapper_.eLast_ = diff;

    model_.setGravity(0,0,0);
    // Warning: The Model needs RADIAN !!!
    for(double& val : desiredPos){
        val *= M_PI / 180.0;
    }
    for(double& val : desiredVel){
        val *= M_PI / 180.0;
    }
    for(double& val : desiredAcc){
        val *= M_PI / 180.0;
    }
    model_.getTorques(desiredPos, desiredVel, desiredAcc, desiredTorques);


    //    std::cout << "model torque :  ";
    //    for(auto d: desiredTorques){
    //        std::cout << d << "\t";
    //    }
    //    std::cout  << std::endl;

    AngularInfo P = (gainP_ * diff).getAngularInfo();
    AngularInfo I = (gainI_ * trajectoryWrapper_.eSum_).getAngularInfo();
    AngularInfo D = (gainD_ * d_diff).getAngularInfo();
    AngularInfo cmd = desiredTorques + P + I + D;


    // We will command torques in Nm!
    cmd_ = cmd;

    joint_state.header.stamp = ros::Time::now();
    joint_state.position = desiredPos;
    joint_state.velocity = desiredVel;
    joint_state.effort = {cmd.Actuator1, cmd.Actuator2, cmd.Actuator3, cmd.Actuator4, cmd.Actuator5, cmd.Actuator6};
    joint_state2 = joint_state;
    joint_state2.effort = desiredTorques;


}

void TorqueTrajectoryController::getDesiredPosition(double dt, AngularInfo& result)
{
    result.Actuator1 = trajectoryWrapper_.jointPosition(dt,0);
    result.Actuator2 = trajectoryWrapper_.jointPosition(dt,1);
    result.Actuator3 = trajectoryWrapper_.jointPosition(dt,2);
    result.Actuator4 = trajectoryWrapper_.jointPosition(dt,3);
    result.Actuator5 = trajectoryWrapper_.jointPosition(dt,4);
    result.Actuator5 = trajectoryWrapper_.jointPosition(dt,5);
}

void TorqueTrajectoryController::getDesiredVelocity(double dt, AngularInfo& result)
{
    result.Actuator1 = trajectoryWrapper_.jointVelocity(dt,0);
    result.Actuator2 = trajectoryWrapper_.jointVelocity(dt,1);
    result.Actuator3 = trajectoryWrapper_.jointVelocity(dt,2);
    result.Actuator4 = trajectoryWrapper_.jointVelocity(dt,3);
    result.Actuator5 = trajectoryWrapper_.jointVelocity(dt,4);
    result.Actuator5 = trajectoryWrapper_.jointVelocity(dt,5);
}

void TorqueTrajectoryController::getDesiredAcceleration(double dt, AngularInfo& result)
{
    result.Actuator1 = trajectoryWrapper_.jointAcceleration(dt,0);
    result.Actuator2 = trajectoryWrapper_.jointAcceleration(dt,1);
    result.Actuator3 = trajectoryWrapper_.jointAcceleration(dt,2);
    result.Actuator4 = trajectoryWrapper_.jointAcceleration(dt,3);
    result.Actuator5 = trajectoryWrapper_.jointAcceleration(dt,4);
    result.Actuator5 = trajectoryWrapper_.jointAcceleration(dt,5);
}

void TorqueTrajectoryController::getDesiredPosition(double dt, std::vector<double>& result)
{
    result.resize(6,0);

    for(std::size_t i = 0; i < 6; ++i){
        result[i] = trajectoryWrapper_.jointPosition(dt, i);
    }
}

void TorqueTrajectoryController::getDesiredVelocity(double dt, std::vector<double>& result)
{
    result.resize(6,0);

    for(std::size_t i = 0; i < 6; ++i){
        result[i] = trajectoryWrapper_.jointVelocity(dt, i);
    }
}

void TorqueTrajectoryController::getDesiredAcceleration(double dt, std::vector<double>& result)
{
    result.resize(6,0);

    for(std::size_t i = 0; i < 6; ++i){
        result[i] = trajectoryWrapper_.jointAcceleration(dt, i);
    }
}


AngularInfo TorqueTrajectoryController::getJointError() const
{
    return trajectoryWrapper_.getJointError();
}

void TorqueTrajectoryController::setGainP(const ManipulatorInfo& gains)
{
    gainP_ = gains;
}

void TorqueTrajectoryController::setGainI(const ManipulatorInfo &gains)
{
    gainI_ = gains;
}

void TorqueTrajectoryController::setGainD(const ManipulatorInfo &gains)
{
    gainD_ = gains;
}

void TorqueTrajectoryController::setConfig(jaco2_driver::jaco2_driver_configureConfig &cfg)
{
    gainP_[0] = cfg.trajectory_p_gain_joint_0;
    gainP_[1] = cfg.trajectory_p_gain_joint_1;
    gainP_[2] = cfg.trajectory_p_gain_joint_2;
    gainP_[3] = cfg.trajectory_p_gain_joint_3;
    gainP_[4] = cfg.trajectory_p_gain_joint_4;
    gainP_[5] = cfg.trajectory_p_gain_joint_5;

    gainI_[0] = cfg.trajectory_i_gain_joint_0;
    gainI_[1] = cfg.trajectory_i_gain_joint_1;
    gainI_[2] = cfg.trajectory_i_gain_joint_2;
    gainI_[3] = cfg.trajectory_i_gain_joint_3;
    gainI_[4] = cfg.trajectory_i_gain_joint_4;
    gainI_[5] = cfg.trajectory_i_gain_joint_5;

    gainD_[0] = cfg.trajectory_d_gain_joint_0;
    gainD_[1] = cfg.trajectory_d_gain_joint_1;
    gainD_[2] = cfg.trajectory_d_gain_joint_2;
    gainD_[3] = cfg.trajectory_d_gain_joint_3;
    gainD_[4] = cfg.trajectory_d_gain_joint_4;
    gainD_[5] = cfg.trajectory_d_gain_joint_5;
}
