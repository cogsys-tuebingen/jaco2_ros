#include <jaco2_kin_dyn_lib/jaco2_residual_vector.h>
Jaco2ResidualVector::Jaco2ResidualVector()
    : model_()
{

}

Jaco2ResidualVector::Jaco2ResidualVector(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip)
    : model_(robot_model, chain_root, chain_tip)
{

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

std::size_t Jaco2ResidualVector::getNrOfJoints()const
{
    model_.getNrOfJoints();
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

void Jaco2ResidualVector::getResidualVector(std::vector<ResidualData> &sequence, std::vector<std::vector<double> > &residual_vec) const
{
    std::vector<Eigen::VectorXd> res;
    getResidualVector(sequence, res);
    residual_vec.resize(sequence.size());
    auto it_res_vec = residual_vec.begin();
    for(auto data : res){
        eigenVector2vector(data, *it_res_vec);
    }
}

void Jaco2ResidualVector::getResidualVector(const ResidualData &data,
                                            const Eigen::VectorXd& last_residual,
                                            const Eigen::VectorXd& last_integral,
                                            Eigen::VectorXd& new_integral,
                                            Eigen::VectorXd& new_residual)
{
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
    Eigen::VectorXd to_integrate = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - G + last_residual;

    new_integral = integration_step(data.dt, last_integral, to_integrate);

    new_residual = gains_ * (m - new_integral);

}

Eigen::VectorXd Jaco2ResidualVector::integration_step(const double dt, const Eigen::VectorXd &last_integral, const Eigen::VectorXd &next_integrant) const
{
    Eigen::VectorXd result = last_integral + dt * next_integrant;
    return result;
}

int Jaco2ResidualVector::getAcceleration(const std::vector<double> &q,
                                         const std::vector<double> &q_Dot,
                                         const std::vector<double> &q_DotDot,
                                         std::vector<std::string> &links,
                                         std::vector<KDL::Twist> &spatial_acc)
{
    return model_.getAcceleration(q, q_Dot, q_DotDot, links, spatial_acc);
}

std::vector<std::string> Jaco2ResidualVector::getLinkNames() const
{
    return model_.getLinkNames();
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

void Jaco2ResidualVector::eigenVector2vector(const Eigen::VectorXd &vec, std::vector<double> &res)
{
    res.resize(vec.rows());
    for(std::size_t i = 0; i < vec.rows(); ++i){
        res[i] = vec(i);
    }
}
