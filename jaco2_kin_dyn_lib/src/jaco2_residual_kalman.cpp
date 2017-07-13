#include <jaco2_kin_dyn_lib/jaco2_residual_kalman.h>
#include <jaco2_kin_dyn_lib/kdl_conversion.h>
#include <Eigen/Dense>
Jaco2ResidualKalman::Jaco2ResidualKalman():
    n_(0)
{

}

void Jaco2ResidualKalman::setDimensions(std::size_t n)
{
    n_ = n;
    eye_ = Eigen::MatrixXd::Identity(n,n);
}
void Jaco2ResidualKalman::initialize(std::size_t n, const Eigen::VectorXd& state)
{
    state_ = state;
    n_ = n;
    eye_ = Eigen::MatrixXd::Identity(n_,n_);
    cov_ = Eigen::MatrixXd::Identity(n_,n_);
}

void Jaco2ResidualKalman::initialize(const std::vector<double> &state)
{
    Jaco2KinDynLib::convert(state, state_);
    n_ = state.size();
    eye_ = Eigen::MatrixXd::Identity(n_,n_);
    cov_ = Eigen::MatrixXd::Identity(n_,n_);
}

void Jaco2ResidualKalman::setCovarianceProcess(const Eigen::MatrixXd& Q)
{
    cov_state_ = Q;
}

void Jaco2ResidualKalman::setCovarianceMeasurment(const Eigen::MatrixXd &R)
{
    cov_measurment_ = R;
}

Eigen::VectorXd Jaco2ResidualKalman::update(Eigen::VectorXd& measurement)
{

    Eigen::MatrixXd cov_pri = cov_ + cov_measurment_;

    Eigen::MatrixXd K = cov_pri * (cov_pri + cov_state_).inverse();
    state_ += K *(measurement - state_);
    cov_ = (eye_ - K)*cov_pri;

    return state_;

}

void Jaco2ResidualKalman::update(const std::vector<double>& measurement, std::vector<double>& result)
{
    Eigen::VectorXd meas;
    Jaco2KinDynLib::convert(measurement, meas);
    Eigen::VectorXd res = update(meas);
    Jaco2KinDynLib::convert(res, result);
}

Eigen::MatrixXd Jaco2ResidualKalman::getCovariance() const
{
    return cov_;
}
