#include <jaco2_driver/point_2_point_velocity_controller.h>


Point2PointVelocityController::Point2PointVelocityController(Jaco2State &state, Jaco2API& api)
    : VelocityController(state, api),
      current_point_(0),
      done_(false)
{
    tp_.InitStruct();
    tp_.Position.Type = ANGULAR_VELOCITY;
}

void Point2PointVelocityController::setTrajectory(const JointTrajectory& trajectory)
{
    trajectory_ = trajectory;
    paramsConst_.resize(trajectory_.size());
    paramsLinear_.resize(trajectory_.size());
    paramsSquare_.resize(trajectory_.size());
    paramsCube_.resize(trajectory_.size());
    paramsQuad_.resize(trajectory_.size());
    timeDiff_.resize(trajectory_.size());
    timeDiff_[0] = 0;

    for(std::size_t i = 1; i < trajectory_.size() -1 ; ++i) // All trajectory points except first and last (start, end)
    {
        double tDiff = trajectory_.getTimeFromStart(i) - trajectory_.getTimeFromStart(i-1);
        double tDiff2 = tDiff*tDiff;
        double tDiff3 = tDiff*tDiff2;
        double tDiff4 = tDiff*tDiff3;
        timeDiff_[i] = tDiff;
        for(std::size_t j = 0; j <trajectory_.numJoints(); ++j) // All joints
        {
            double th0 = trajectory_.getPosition(i-1,j);
            double v0 = trajectory_.getVelocity(i-1,j);
            double a0 = trajectory_.getAcceleration(i-1,j);
            double thn = trajectory_.getPosition(i,j);
            double vn = trajectory_.getVelocity(i,j);
            double an = trajectory_.getAcceleration(i,j);
            paramsConst_[i][j]  = v0;
            paramsLinear_[i][j] = a0;
            paramsSquare_[i][j] = -(3 * (tDiff * (12 * v0 + 8 * vn + 3 * a0 * tDiff - an * tDiff) + 20 *(th0 - thn)))/(tDiff3);
            paramsCube_[i][j]   = (6 * (16 * v0 * tDiff + 14 * vn * tDiff + 3 * a0 * tDiff2 - 2 * an * tDiff2 + 30 * th0 - 30 * thn))/tDiff4;
            paramsQuad_[i][j]   = -(10 * (6 * v0 * tDiff + 6 * vn * tDiff + a0 * tDiff2 - an * tDiff2 + 12 * th0 - 12 * thn))/(tDiff *tDiff4);
        }
    }
    done_ = false;
    start_command_ = std::time(nullptr);
}

void Point2PointVelocityController::write()
{
    std::time_t now = std::time(nullptr);
    double timeDiff = now - start_command_ ;
    double dt = timeDiff - timeDiff_[current_point_];
    if(timeDiff > trajectory_.getTimeFromStart(current_point_))
    {
        ++current_point_;
    }
    if(current_point_ == trajectory_.size())
    {
        done_ = true;
    }
    else if(timeDiff <= trajectory_.getTimeFromStart(current_point_))
    {
        ManipulatorInfo cmdVel;
        for(std::size_t i = 0; i < cmdVel.length_; ++ i)
        {
            cmdVel[i] = paramsConst_[current_point_][i] + paramsLinear_[current_point_][i] * dt + 0.5 * paramsCube_[current_point_][i] * dt * dt +
                    paramsCube_[current_point_][i]/3.0 * dt * dt * dt + paramsQuad_[current_point_][i]/4.0 * dt * dt * dt  * dt;
        }
        tp_.Position.Actuators.Actuator1 = cmdVel[0];
        tp_.Position.Actuators.Actuator2 = cmdVel[1];
        tp_.Position.Actuators.Actuator3 = cmdVel[2];
        tp_.Position.Actuators.Actuator4 = cmdVel[3];
        tp_.Position.Actuators.Actuator5 = cmdVel[4];
        tp_.Position.Actuators.Actuator6 = cmdVel[5];
        api_.setAngularVelocity(tp_);
    }
    else
    {
        bool found = false;
        for(std::size_t i = current_point_; i < trajectory_.size(); ++ i)
        {
            if(timeDiff + 0.01 < trajectory_.getTimeFromStart(i))
            {
                current_point_ = i;
                found = true;
                break;
            }
        }
        if(!found)
        {
            done_ = true;
        }
    }

}

bool Point2PointVelocityController::isDone() const
{
    return done_;
}
