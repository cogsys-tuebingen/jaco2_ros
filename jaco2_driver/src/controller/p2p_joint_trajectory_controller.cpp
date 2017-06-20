#include <jaco2_driver/controller/p2p_joint_trajectory_controller.h>

P2PJointTrajactoryController::P2PJointTrajactoryController():
    current_point_(0)
{

}

void P2PJointTrajactoryController::setTrajectory(const JointTrajectory& trajectory)
{
    trajectory_ = trajectory;
    paramsConst_.clear();
    paramsConst_.resize(trajectory_.size());
    paramsLinear_.clear();
    paramsLinear_.resize(trajectory_.size());
    paramsSquare_.clear();
    paramsSquare_.resize(trajectory_.size());
    paramsCube_.clear();
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
    //    done_ = false;
    current_point_ = 0;
    start_command_ = std::chrono::high_resolution_clock::now();
    last_command_ = start_command_;

}

double P2PJointTrajactoryController::jointAcceleration(const double dt, const std::size_t joint) const
{
    return paramsSquare_[current_point_][joint] + 2.0 * paramsCube_[current_point_][joint] * dt;
}

double P2PJointTrajactoryController::jointCmdVelocity(const double dt, const std::size_t joint) const
{
    double dt2 = dt*dt;

    return paramsLinear_[current_point_][joint] +
            paramsSquare_[current_point_][joint] * dt +
            paramsCube_[current_point_][joint] * dt2;
}

double P2PJointTrajactoryController::jointPosition(const double dt, const std::size_t joint) const
{
    double dt2 = dt*dt;
    double dt3 = dt*dt2;
    return paramsConst_[current_point_][joint] +
            paramsLinear_[current_point_][joint] * dt +
            0.5*paramsSquare_[current_point_][joint] * dt2 +
            paramsCube_[current_point_][joint]/3.0 * dt3 ;
}

void P2PJointTrajactoryController::setGainP(const ManipulatorInfo& gains)
{
    gainP_ = gains;
}

void P2PJointTrajactoryController::setGainI(const ManipulatorInfo &gains)
{
    gainI_ = gains;
}

void P2PJointTrajactoryController::setGainD(const ManipulatorInfo &gains)
{
    gainD_ = gains;
}

void P2PJointTrajactoryController::setConfig(jaco2_driver::jaco2_driver_configureConfig &cfg)
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

AngularInfo P2PJointTrajactoryController::getJointError() const
{
    AngularInfo error;
    error.InitStruct();
    if(current_point_ < trajectory_.size())
    {
        error = posDiff_[current_point_].toAngularInfo();
    }
    return error;
}

ManipulatorInfo P2PJointTrajactoryController::diffTrajectoryPoint(AngularInfo& current_position)
{

//    auto current_position =  jstate_.getAngularPosition();
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


void P2PJointTrajactoryController::evaluationOutput()
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
    std::string delimiter("\t");
    ManipulatorInfo totalError = ManipulatorInfo::sum(rootSquaredDiff);
    ManipulatorInfo standardDev = ManipulatorInfo::elment_sqrt(ManipulatorInfo::variance(posDiff_,meanError));
    std::cout << "'sum of the root of the squared differences (SRS) /total error''" << std::endl << totalError.toString(delimiter) << std::endl;
    ManipulatorInfo meanSRS = ManipulatorInfo::mean(rootSquaredDiff);
    std::cout << "'mean SRS'  " << std::endl <<  meanSRS.toString(delimiter) << std::endl;
    std::cout << "'std SRS' " << std::endl <<  ManipulatorInfo::elment_sqrt(ManipulatorInfo::variance(rootSquaredDiff,meanSRS)).toString(delimiter) << std::endl;
    std::cout << "'meanErrors' " << std::endl << meanError.toString(delimiter) << std::endl;
    std::cout << "'Standard deviation' " << std::endl << standardDev.toString(delimiter) << std::endl;
    std::cout << "'maxErrors' " << std::endl << maxError.toString(delimiter) << std::endl;
    std::cout << "'endPointDifference' " << std::endl << posDiff_.back().toString(delimiter) << std::endl;
    ManipulatorInfo vmax;
    ManipulatorInfo::max(ManipulatorInfo::abs(paramsLinear_),vmax);
    std::cout << "'max abs(velocity)' "  << std::endl<< vmax.toString(delimiter) << std::endl;
    std::cout << std::endl;
}
