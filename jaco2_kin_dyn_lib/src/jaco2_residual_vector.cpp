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

void Jaco2ResidualVector::getResidualVector(std::vector<ResidualData> &sequence, std::vector<double> &residual_vec) const
{
    // initialization
    std::size_t n_joints =  model_.getNrOfJoints();
    std::vector<Eigen::VectorXd> vec_m(sequence.size());
    std::vector<Eigen::VectorXd> model_coeff(sequence.size());
    std::vector<Eigen::VectorXd> r_old(sequence.size());
    std::vector<Eigen::VectorXd> r_next(sequence.size());

    auto it_model_coeff = model_coeff.begin();
    auto it_vec_m = vec_m.begin();
    auto it_r_old = r_next.begin();

    for(auto data : sequence){
        (*it_r_old).setZero(n_joints);
        ++it_r_old;

        Eigen::MatrixXd C;
        model_.getMatrixC(data.joint_positions,data.joint_velocities, C);

        Eigen::VectorXd omega;
        Eigen::VectorXd tau;
        vector2EigenVector(data.joint_velocities, omega);
        vector2EigenVector(data.torques, tau);

        Eigen::MatrixXd H;
        Eigen::VectorXd G;
        model_.getChainDynInertiaAndGravity(data.gx, data.gy, data.gz, data.joint_positions, H, G);
        *it_vec_m = H*omega;
        ++it_vec_m;

        *it_model_coeff  = tau + Eigen::Transpose<Eigen::MatrixXd>(C) * omega - G;
        ++it_model_coeff;
    }

    //self consistant loop
    double diff = 1e10;
    std::size_t iterations = 0;

    while(diff > accuracy_ || iterations > max_iter_){
        r_old = r_next;
        iteration(sequence, model_coeff, r_old,vec_m, r_next);
        diff = std::abs((r_next.back() - r_old.back()).sum());
        ++iterations;
    }


}

void Jaco2ResidualVector::iteration(const std::vector<ResidualData> &sequence,
                                    const std::vector<Eigen::VectorXd>& model_coeff,
                                    const std::vector<Eigen::VectorXd>& r_old,
                                    const std::vector<Eigen::VectorXd>& vec_m,
                                    std::vector<Eigen::VectorXd> &res) const
{
    auto it_r_old = r_old.begin();
    auto it_model_coeff = model_coeff.begin();
    auto it_vec_m = vec_m.begin();
    auto it_r_next = res.begin();
    for(std::size_t i  = 0; i < sequence.size(); ++i){
        Eigen::VectorXd integral;
        integrate(i, sequence, model_coeff, r_old, integral);
        *it_r_next = gains_ * ((*it_vec_m) - integral);
    }

}
void Jaco2ResidualVector::integrate(const std::size_t iter,
                                    const std::vector<ResidualData> &sequence,
                                    const std::vector<Eigen::VectorXd>& model_coeff,
                                    const std::vector<Eigen::VectorXd>& r_old,
                                    Eigen::VectorXd &res) const
{


    //TODO Implement


//    res.resize(f.size(),0);
//    auto t_i = stamp.begin();
//    auto f_i = f.begin();
//    auto res_i = f.begin();
//    *res_i = 0;
//    ++res_i;

//    for(res_i; res_i < res.end(); ++res_i){
//        double t_pre =*t_i;
//        ++t_i;
//        *res_i = (*t_i - t_pre) * (*f_i);
//        ++f_i;
//    }
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
