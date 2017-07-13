#ifndef JACO2_RESIDUAL_KALMAN_H
#define JACO2_RESIDUAL_KALMAN_H

#include <vector>
#include <Eigen/Core>
class Jaco2ResidualKalman
{
public:
    Jaco2ResidualKalman();


    void initialize(std::size_t n, const Eigen::VectorXd& state);
    void initialize(const std::vector<double> & state);
    void setDimensions(std::size_t n);
    void setCovarianceProcess(const Eigen::MatrixXd& Q);
    void setCovarianceMeasurment(const Eigen::MatrixXd& R);

    Eigen::VectorXd update(Eigen::VectorXd& measurement);
    void update(const std::vector<double> &measurement, std::vector<double> &result);

    Eigen::MatrixXd getCovariance() const;

private:
    std::size_t n_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd cov_state_;
    Eigen::MatrixXd cov_measurment_;
    Eigen::MatrixXd cov_;
    Eigen::MatrixXd eye_;

};

#endif // JACO2_RESIDUAL_KALMAN_H
