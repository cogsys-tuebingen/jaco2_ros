#include <jaco2_driver/point_2_point_velocity_controller.h>
#include <math.h>

Point2PointVelocityController::Point2PointVelocityController(Jaco2State &state, Jaco2API& api)
    : VelocityController(state, api),
      current_point_(0),
      done_(false)
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
    api_.disableTorque();
}

void Point2PointVelocityController::setTrajectory(const JointTrajectory& trajectory)
{
    trajectory_ = trajectory;
    paramsConst_.resize(trajectory_.size());
    paramsLinear_.resize(trajectory_.size());
    paramsSquare_.resize(trajectory_.size());
    paramsCube_.resize(trajectory_.size());
    timeDiff_.resize(trajectory_.size());
    posDiff_.resize(trajectory_.size());
    timeDiff_[0] = 0;
    for(std::size_t i = 1; i < trajectory_.size() ; ++i) // All trajectory points except first (start)
    {
        double tDiff = trajectory_.getTimeFromStart(i) - trajectory_.getTimeFromStart(i-1);
        double tDiff2 = tDiff*tDiff;
        double tDiff3 = tDiff*tDiff2;
        timeDiff_[i] = tDiff;
        for(std::size_t j = 0; j <trajectory_.numJoints(); ++j) // All joints
        {
            double th0 = trajectory_.getPosition(i-1,j);
            double v0 = trajectory_.getVelocity(i-1,j);
            double thn = trajectory_.getPosition(i,j);
            double vn = trajectory_.getVelocity(i,j);
            paramsConst_[i][j]  = th0;
            paramsLinear_[i][j] = v0;
            paramsSquare_[i][j] = 2.0/tDiff2*( 3.0*(thn-th0) - tDiff*(vn+2.0*v0));
            paramsCube_[i][j] = 3.0/tDiff3*(-2.0*(thn-th0) + tDiff*(vn+v0));
        }
    }
    paramsConst_.push_back(paramsConst_.back());
    paramsLinear_.push_back(paramsLinear_.back());
    paramsSquare_.push_back(paramsSquare_.back());
    for (std::size_t i = 0; i < eLast_.length_; ++i)
    {
        eLast_[i] = 0;
        eSum_[i] = 0;
    }
    done_ = false;
    current_point_ = 0;
    start_command_ = std::chrono::high_resolution_clock::now();

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

double Point2PointVelocityController::jointCmdVelocity(const double dt, const std::size_t joint) const
{
    double dt2 = dt*dt;

    return paramsLinear_[current_point_][joint] + paramsSquare_[current_point_][joint] * dt +
           paramsCube_[current_point_][joint] * dt2;
}

double Point2PointVelocityController::jointPosition(const double dt, const std::size_t joint) const
{
    double dt2 = dt*dt;
    double dt3 = dt*dt2;
    return paramsConst_[current_point_][joint] + paramsLinear_[current_point_][joint] * dt +
           0.5*paramsSquare_[current_point_][joint] * dt2 +
           paramsCube_[current_point_][joint]/3.0 * dt3 ;
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

ManipulatorInfo Point2PointVelocityController::diffTrajectoryPoint()
{

    auto current_position =  state_.getAngularPosition();
    ManipulatorInfo diff;
    diff[0] = fabs(current_position.Actuators.Actuator1 - trajectory_.getPosition(current_point_,0));
    diff[1] = fabs(current_position.Actuators.Actuator2 - trajectory_.getPosition(current_point_,1));
    diff[2] = fabs(current_position.Actuators.Actuator3 - trajectory_.getPosition(current_point_,2));
    diff[3] = fabs(current_position.Actuators.Actuator4 - trajectory_.getPosition(current_point_,3));
    diff[4] = fabs(current_position.Actuators.Actuator5 - trajectory_.getPosition(current_point_,4));
    diff[5] = fabs(current_position.Actuators.Actuator6 - trajectory_.getPosition(current_point_,5));

    diff.normalizeAngleDegrees();

    return diff;
}

void Point2PointVelocityController::setGainP(const ManipulatorInfo& gains)
{
    gainP_ = gains;
//    for(std::size_t i = 0; i < gains.length_; ++i)
//    {
//        std::cout << gains[i] << " | ";
//    }
//    std::cout << std::endl;
}

void Point2PointVelocityController::setGainI(const ManipulatorInfo &gains)
{
    gainI_ = gains;
}

void Point2PointVelocityController::setGainD(const ManipulatorInfo &gains)
{
    gainD_ = gains;
}

AngularInfo Point2PointVelocityController::getJointError() const
{
    AngularInfo error;
    error.InitStruct();
    if(current_point_ < trajectory_.size())
    {
        error = posDiff_[current_point_].toAngularInfo();
    }
    return error;
}


void Point2PointVelocityController::evaluationOutput()
{
    std::cout << "posDiff_.size(): " <<  posDiff_.size() << " | trajectory_.size(): " << trajectory_.size() << std::endl;
    ManipulatorInfo meanError = ManipulatorInfo::mean(posDiff_);
    ManipulatorInfo maxError;
    ManipulatorInfo::max(posDiff_,maxError);
    std::vector<ManipulatorInfo> rootSquaredDiff;
    for(auto diff : posDiff_)
    {
        ManipulatorInfo tmp = ManipulatorInfo::elment_sqrt(diff * diff);
        rootSquaredDiff.push_back(tmp);

    }
    std::string delimiter(" | ");
    ManipulatorInfo totalError = ManipulatorInfo::sum(rootSquaredDiff);
    ManipulatorInfo standardDev = ManipulatorInfo::elment_sqrt(ManipulatorInfo::variance(posDiff_,meanError));
    std::cout << "'sum of the root of the squared differences (SRS) /total error'' | " << totalError.toString(delimiter) << std::endl;
    ManipulatorInfo meanSRS = ManipulatorInfo::mean(rootSquaredDiff);
    std::cout << "'mean SRS' | " <<  meanSRS.toString(delimiter) << std::endl;
    std::cout << "'std SRS' | " <<  ManipulatorInfo::elment_sqrt(ManipulatorInfo::variance(rootSquaredDiff,meanSRS)).toString(delimiter) << std::endl;
    std::cout << "'meanErrors' | " << meanError.toString(delimiter) << std::endl;
    std::cout << "'Standard deviation' | " << standardDev.toString(delimiter) << std::endl;
    std::cout << "'maxErrors' : " << maxError.toString(delimiter) << std::endl;
    std::cout << "'endPointDifference' | " << posDiff_.back().toString(delimiter) << std::endl;
    ManipulatorInfo vmax;
    ManipulatorInfo::max(paramsLinear_,vmax);
    std::cout << "'max velocity' | " << vmax.toString(delimiter) << std::endl;
    std::cout << std::endl;
}
