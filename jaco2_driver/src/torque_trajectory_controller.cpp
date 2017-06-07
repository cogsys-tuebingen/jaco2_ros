#include <jaco2_driver/torque_trajectory_controller.h>
#include <kinova/KinovaArithmetics.hpp>

TorqueTrajectoryController::TorqueTrajectoryController(Jaco2State &state, Jaco2API &api):
    Point2PointVelocityController(state, api),
    model_("/robot_description","jaco_link_base","jaco_link_hand"),
    nh("~")
{
    pub = nh.advertise<sensor_msgs::JointState>("/torque_control_cmd",2);
    pub2 = nh.advertise<sensor_msgs::JointState>("/torque_control_cmd2",2);
}

void TorqueTrajectoryController::setTrajectory(const JointTrajectory& trajectory)
{
    api_.enableDirectTorqueMode(1.0,0.5);
    Point2PointVelocityController::setTrajectory(trajectory);
}


void TorqueTrajectoryController::start()
{
    api_.enableDirectTorqueMode(1.0,0.5);
    //    done_

    done_ = false;
    //    std::cout << "start torque control" << std::endl;
}


void TorqueTrajectoryController::write()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - start_command_ ;
    auto durationLast = now -last_command_;
    last_command_ = now;
    double timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6;
    samplingPeriod_ = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;

    while(timeDiff > trajectory_.getTimeFromStart(current_point_))
    {
        ++current_point_;
        if(current_point_ >= trajectory_.size() )
        {
            std::cout << "done " << std::endl;
            done_ = true;
            // publish zero velocity
            tp_.InitStruct();
            tp_.Position.Type = ANGULAR_VELOCITY;
            //            api_.disableTorque();
            evaluationOutput();
            //            api_.setAngularTorque(tp_.Position.Actuators);
            api_.disableTorque();
            return;
        }
    }
    if(fabs(timeDiff -  trajectory_.getTimeFromStart(current_point_)) < 0.02)
    {
        ManipulatorInfo diff = diffTrajectoryPoint();
        posDiff_[current_point_] = diff;

    }
    if(current_point_ <= 0)
    {
        return;
    }
    double dt = timeDiff - trajectory_.getTimeFromStart(current_point_-1);
    if(current_point_ < trajectory_.size())
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
        api_.setAngularTorque(tp_.Position.Actuators);

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
    eSum_ = diff * samplingPeriod_;
    eLast_ = diff;

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
    AngularInfo I = (gainI_ * eSum_).getAngularInfo();
    AngularInfo D = (gainD_ * d_diff).getAngularInfo();
    AngularInfo cmd =desiredTorques + P + I + D;


    // We will command torques in Nm!
    tp_.Position.Actuators = cmd;

    joint_state.header.stamp = ros::Time::now();
    joint_state.position = desiredPos;
    joint_state.velocity = desiredVel;
    joint_state.effort = {cmd.Actuator1, cmd.Actuator2, cmd.Actuator3, cmd.Actuator4, cmd.Actuator5, cmd.Actuator6};
    joint_state2 = joint_state;
    joint_state2.effort = desiredTorques;


}

void TorqueTrajectoryController::getDesiredPosition(double dt, AngularInfo& result)
{
    result.Actuator1 = jointPosition(dt,0);
    result.Actuator2 = jointPosition(dt,1);
    result.Actuator3 = jointPosition(dt,2);
    result.Actuator4 = jointPosition(dt,3);
    result.Actuator5 = jointPosition(dt,4);
    result.Actuator5 = jointPosition(dt,5);
}

void TorqueTrajectoryController::getDesiredVelocity(double dt, AngularInfo& result)
{
    result.Actuator1 = jointCmdVelocity(dt,0);
    result.Actuator2 = jointCmdVelocity(dt,1);
    result.Actuator3 = jointCmdVelocity(dt,2);
    result.Actuator4 = jointCmdVelocity(dt,3);
    result.Actuator5 = jointCmdVelocity(dt,4);
    result.Actuator5 = jointCmdVelocity(dt,5);
}

void TorqueTrajectoryController::getDesiredAcceleration(double dt, AngularInfo& result)
{
    result.Actuator1 = jointAcceleration(dt,0);
    result.Actuator2 = jointAcceleration(dt,1);
    result.Actuator3 = jointAcceleration(dt,2);
    result.Actuator4 = jointAcceleration(dt,3);
    result.Actuator5 = jointAcceleration(dt,4);
    result.Actuator5 = jointAcceleration(dt,5);
}

void TorqueTrajectoryController::getDesiredPosition(double dt, std::vector<double>& result)
{
    result.resize(6,0);

    for(std::size_t i = 0; i < 6; ++i){
        result[i] = jointPosition(dt, i);
    }
}

void TorqueTrajectoryController::getDesiredVelocity(double dt, std::vector<double>& result)
{
    result.resize(6,0);

    for(std::size_t i = 0; i < 6; ++i){
        result[i] = jointCmdVelocity(dt, i);
    }
}

void TorqueTrajectoryController::getDesiredAcceleration(double dt, std::vector<double>& result)
{
    result.resize(6,0);

    for(std::size_t i = 0; i < 6; ++i){
        result[i] = jointAcceleration(dt, i);
    }
}
