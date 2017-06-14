#include <jaco2_driver/controller/point_2_point_velocity_controller.h>
#include <math.h>

Point2PointVelocityController::Point2PointVelocityController(Jaco2State &state, Jaco2API& api)
    : P2PJointTrajactoryController(state, api)
{
    tp_.InitStruct();
    tp_.Position.Type = ANGULAR_VELOCITY;

    gainP_[0] = 11.29063216;
    gainP_[1] = 11.29063216;
    gainP_[2] = 11.29063216;
    gainP_[3] = 11.29063216;
    gainP_[4] = 11.29063216;
    gainP_[5] = 11.29063216;
}

void Point2PointVelocityController::start()
{
}

void Point2PointVelocityController::write()
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
            api_.setAngularVelocity(tp_);
            evaluationOutput();
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
    ManipulatorInfo diff;
    ManipulatorInfo d_diff;
    diff[0] = (jointPosition(dt,0) - currentPos.Actuators.Actuator1);
    diff[1] = (jointPosition(dt,1) - currentPos.Actuators.Actuator2);
    diff[2] = (jointPosition(dt,2) - currentPos.Actuators.Actuator3);
    diff[3] = (jointPosition(dt,3) - currentPos.Actuators.Actuator4);
    diff[4] = (jointPosition(dt,4) - currentPos.Actuators.Actuator5);
    diff[5] = (jointPosition(dt,5) - currentPos.Actuators.Actuator6);
    diff.normalizeAngleDegrees();

    for(std::size_t i = 0; i <diff.length_; ++i)
    {
        eSum_[i] += samplingPeriod_ * diff[i];
        d_diff[i] = (diff[i] - eLast_[i])/samplingPeriod_;
        eLast_[i] = diff[i];
    }


    tp_.Position.Actuators.Actuator1 = gainP_[0] * diff[0] + gainI_[0] * eSum_[0] + gainD_[0] * d_diff[0];
    tp_.Position.Actuators.Actuator2 = gainP_[1] * diff[1] + gainI_[1] * eSum_[1] + gainD_[1] * d_diff[1];
    tp_.Position.Actuators.Actuator3 = gainP_[2] * diff[2] + gainI_[2] * eSum_[2] + gainD_[2] * d_diff[2];
    tp_.Position.Actuators.Actuator4 = gainP_[3] * diff[3] + gainI_[3] * eSum_[3] + gainD_[3] * d_diff[3];
    tp_.Position.Actuators.Actuator5 = gainP_[4] * diff[4] + gainI_[4] * eSum_[4] + gainD_[4] * d_diff[4];
    tp_.Position.Actuators.Actuator6 = gainP_[5] * diff[5] + gainI_[5] * eSum_[5] + gainD_[5] * d_diff[5];
    tp_.Position.Type = ANGULAR_VELOCITY;
}

void Point2PointVelocityController::simpleVelController(const double dt)
{
    AngularPosition currentVel = state_.getAngularVelocity();
    tp_.Position.Actuators.Actuator1 = currentVel.Actuators.Actuator1 + gainP_[0]*(jointCmdVelocity(dt,0) - currentVel.Actuators.Actuator1);
    tp_.Position.Actuators.Actuator2 = currentVel.Actuators.Actuator2 + gainP_[1]*(jointCmdVelocity(dt,1) - currentVel.Actuators.Actuator2);
    tp_.Position.Actuators.Actuator3 = currentVel.Actuators.Actuator3 + gainP_[2]*(jointCmdVelocity(dt,2) - currentVel.Actuators.Actuator3);
    tp_.Position.Actuators.Actuator4 = currentVel.Actuators.Actuator4 + gainP_[3]*(jointCmdVelocity(dt,3) - currentVel.Actuators.Actuator4);
    tp_.Position.Actuators.Actuator5 = currentVel.Actuators.Actuator5 + gainP_[4]*(jointCmdVelocity(dt,4) - currentVel.Actuators.Actuator5);
    tp_.Position.Actuators.Actuator6 = currentVel.Actuators.Actuator6 + gainP_[5]*(jointCmdVelocity(dt,5) - currentVel.Actuators.Actuator6);
    tp_.Position.Type = ANGULAR_VELOCITY;
}

