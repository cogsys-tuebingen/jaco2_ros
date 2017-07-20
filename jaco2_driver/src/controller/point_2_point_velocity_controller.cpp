#include <jaco2_driver/controller/point_2_point_velocity_controller.h>
#include <math.h>

Point2PointVelocityController::Point2PointVelocityController(Jaco2State &state, Jaco2API& api)
    : TrajectoryTrackingController(state, api)
{
    tp_.InitStruct();
    tp_.Position.Type = ANGULAR_VELOCITY;


    gainP_[0] = 11.29063216;
    gainP_[1] = 11.29063216;
    gainP_[2] = 11.29063216;
    gainP_[3] = 11.29063216;
    gainP_[4] = 11.29063216;
    gainP_[5] = 11.29063216;

    tp_.InitStruct();
}

void Point2PointVelocityController::start()
{
}


void Point2PointVelocityController::setTrajectory(const JointTrajectory &trajectory)
{
    trajectoryWrapper_.setTrajectory(trajectory);
    current_point_ = 0;
    done_ = false;
    start_command_ = std::chrono::high_resolution_clock::now();
    last_command_ = start_command_;
    result_ = ControllerResult::WORKING;
}

void Point2PointVelocityController::write()
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
            tp_.InitStruct();
            tp_.Position.Type = ANGULAR_VELOCITY;
            api_.setAngularVelocity(tp_);
            trajectoryWrapper_.evaluationOutput();
            result_ = ControllerResult::SUCCESS;
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
        pidController(dt);

        api_.setAngularVelocity(tp_);
    }


}

bool Point2PointVelocityController::isDone() const
{
    return done_;
}

void Point2PointVelocityController::pidController(const double dt)
{
    AngularPosition currentPos = state_.getAngularPosition();
    AngularPosition currentVel = state_.getAngularVelocity();
    ManipulatorInfo diff;
    ManipulatorInfo d_diff;
    diff[0] = (trajectoryWrapper_.jointPosition(dt,0) - currentPos.Actuators.Actuator1);
    diff[1] = (trajectoryWrapper_.jointPosition(dt,1) - currentPos.Actuators.Actuator2);
    diff[2] = (trajectoryWrapper_.jointPosition(dt,2) - currentPos.Actuators.Actuator3);
    diff[3] = (trajectoryWrapper_.jointPosition(dt,3) - currentPos.Actuators.Actuator4);
    diff[4] = (trajectoryWrapper_.jointPosition(dt,4) - currentPos.Actuators.Actuator5);
    diff[5] = (trajectoryWrapper_.jointPosition(dt,5) - currentPos.Actuators.Actuator6);
    diff.normalizeAngleDegrees();

    d_diff[0] = (trajectoryWrapper_.jointVelocity(dt,0) - currentVel.Actuators.Actuator1);
    d_diff[1] = (trajectoryWrapper_.jointVelocity(dt,1) - currentVel.Actuators.Actuator2);
    d_diff[2] = (trajectoryWrapper_.jointVelocity(dt,2) - currentVel.Actuators.Actuator3);
    d_diff[3] = (trajectoryWrapper_.jointVelocity(dt,3) - currentVel.Actuators.Actuator4);
    d_diff[4] = (trajectoryWrapper_.jointVelocity(dt,4) - currentVel.Actuators.Actuator5);
    d_diff[5] = (trajectoryWrapper_.jointVelocity(dt,5) - currentVel.Actuators.Actuator6);

    for(std::size_t i = 0; i <diff.length_; ++i)
    {
        trajectoryWrapper_.eSum_[i] += trajectoryWrapper_.samplingPeriod_ * diff[i];
//        d_diff[i] = (diff[i] - eLast_[i])/samplingPeriod_; // either this or above d_diff
        trajectoryWrapper_.eLast_[i] = diff[i];
    }


    tp_.Position.Actuators.Actuator1 = gainP_[0] * diff[0] + gainI_[0] * trajectoryWrapper_.eSum_[0] + gainD_[0] * d_diff[0];
    tp_.Position.Actuators.Actuator2 = gainP_[1] * diff[1] + gainI_[1] * trajectoryWrapper_.eSum_[1] + gainD_[1] * d_diff[1];
    tp_.Position.Actuators.Actuator3 = gainP_[2] * diff[2] + gainI_[2] * trajectoryWrapper_.eSum_[2] + gainD_[2] * d_diff[2];
    tp_.Position.Actuators.Actuator4 = gainP_[3] * diff[3] + gainI_[3] * trajectoryWrapper_.eSum_[3] + gainD_[3] * d_diff[3];
    tp_.Position.Actuators.Actuator5 = gainP_[4] * diff[4] + gainI_[4] * trajectoryWrapper_.eSum_[4] + gainD_[4] * d_diff[4];
    tp_.Position.Actuators.Actuator6 = gainP_[5] * diff[5] + gainI_[5] * trajectoryWrapper_.eSum_[5] + gainD_[5] * d_diff[5];
    tp_.Position.Type = ANGULAR_VELOCITY;
}

void Point2PointVelocityController::simpleVelController(const double dt)
{
    AngularPosition currentVel = state_.getAngularVelocity();
    tp_.Position.Actuators.Actuator1 = gainP_[0]*(trajectoryWrapper_.jointVelocity(dt,0) - currentVel.Actuators.Actuator1);
    tp_.Position.Actuators.Actuator2 = gainP_[1]*(trajectoryWrapper_.jointVelocity(dt,1) - currentVel.Actuators.Actuator2);
    tp_.Position.Actuators.Actuator3 = gainP_[2]*(trajectoryWrapper_.jointVelocity(dt,2) - currentVel.Actuators.Actuator3);
    tp_.Position.Actuators.Actuator4 = gainP_[3]*(trajectoryWrapper_.jointVelocity(dt,3) - currentVel.Actuators.Actuator4);
    tp_.Position.Actuators.Actuator5 = gainP_[4]*(trajectoryWrapper_.jointVelocity(dt,4) - currentVel.Actuators.Actuator5);
    tp_.Position.Actuators.Actuator6 = gainP_[5]*(trajectoryWrapper_.jointVelocity(dt,5) - currentVel.Actuators.Actuator6);
    tp_.Position.Type = ANGULAR_VELOCITY;
}


AngularInfo Point2PointVelocityController::getJointError() const
{
    return trajectoryWrapper_.getJointError();
}

void Point2PointVelocityController::setGainP(const ManipulatorInfo& gains)
{
    gainP_ = gains;
}

void Point2PointVelocityController::setGainI(const ManipulatorInfo &gains)
{
    gainI_ = gains;
}

void Point2PointVelocityController::setGainD(const ManipulatorInfo &gains)
{
    gainD_ = gains;
}

void Point2PointVelocityController::setConfig(jaco2_driver::jaco2_driver_configureConfig &cfg)
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
