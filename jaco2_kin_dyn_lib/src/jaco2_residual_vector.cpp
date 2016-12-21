#include <jaco2_kin_dyn_lib/jaco2_residual_vector.h>
Jaco2ResidualVector::Jaco2ResidualVector()
    : model_(),
      accuracy_(0.1),
      max_iter_(10)
{

}

Jaco2ResidualVector::Jaco2ResidualVector(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip)
    : model_(robot_model, chain_root, chain_tip),
      accuracy_(0.1),
      max_iter_(10)
{

}

void Jaco2ResidualVector::setAccuracy(double val)
{
    accuracy_ = val;
}

void Jaco2ResidualVector::setGains(std::vector<double> &gains)
{
    gains_.resize(gains.size(),gains.size());
    gains_.setZero(gains.size(),gains.size());
    std::size_t i = 0;
    for(auto value : gains){
        gains_(i,i) = value;
        ++i;
    }
}

void Jaco2ResidualVector::setGravity(double x, double y, double z)
{
    model_.setGravity(x,y,z);
}

void Jaco2ResidualVector::setMaxIterations(std::size_t val)
{
    max_iter_ = val;
}

void Jaco2ResidualVector::setTree(const std::string &robot_model)
{
    model_.setTree(robot_model);
}

void Jaco2ResidualVector::setRootAndTip(const std::string &chain_root, const std::string &chain_tip)
{
    model_.setRootAndTip(chain_root, chain_tip);
}

void Jaco2ResidualVector::changeDynamicParams(const std::string &link, const double mass, const Eigen::Vector3d &com, const Eigen::Matrix3d &inertia)
{
    model_.changeDynamicParams(link, mass, com, inertia);
}

void Jaco2ResidualVector::getResidualVector(std::vector<ResidualData> &sequence, std::vector<Eigen::VectorXd> &residual_vec) const
{
    // initialization
    std::size_t n_joints =  model_.getNrOfJoints();
    Eigen::VectorXd r0(n_joints);
    r0.setZero(n_joints);

    residual_vec.resize(sequence.size());

    auto it_residual_vec = residual_vec.begin();
    *it_residual_vec = r0;

    Eigen::VectorXd last_integral(n_joints);
    last_integral.setZero(n_joints);

    for(auto data : sequence){

        Eigen::MatrixXd C;
        model_.getMatrixC(data.joint_positions,data.joint_velocities, C);

        Eigen::VectorXd omega;
        Eigen::VectorXd tau;
        vector2EigenVector(data.joint_velocities, omega);
        vector2EigenVector(data.torques, tau);

        Eigen::MatrixXd H;
        Eigen::VectorXd G;
        model_.getChainDynInertiaAndGravity(data.gx, data.gy, data.gz, data.joint_positions, H, G);
        Eigen::VectorXd m = H * omega;
        Eigen::VectorXd to_integrate = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - G + *it_residual_vec;

        Eigen::VectorXd integral = integration_step(data.dt, last_integral, to_integrate);
        last_integral = integral;

        *it_residual_vec = gains_ * (m - integral);
        ++it_residual_vec;
    }


}


Eigen::VectorXd Jaco2ResidualVector::integration_step(const double dt, const Eigen::VectorXd &last_integral, const Eigen::VectorXd &next_integrant) const
{
    // TODO
}

void Jaco2ResidualVector::vector2EigenVector(const std::vector<double> &vec, Eigen::VectorXd &res)
{
    res.resize(vec.size());
    std::size_t i = 0;
    for(auto data : vec)
    {
        res(i) = data;
        ++i;
    }
}
