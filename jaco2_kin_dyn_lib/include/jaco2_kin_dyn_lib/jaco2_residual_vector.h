#ifndef JACO2RESIDUALVECTOR_H
#define JACO2RESIDUALVECTOR_H
#include <string>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
struct ResidualData
{
    double time_stamp;
    double gx;
    double gy;
    double gz;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> torques;
};


class Jaco2ResidualVector
{
public:
    Jaco2ResidualVector();
    Jaco2ResidualVector(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

    void setAccuracy(double val);
    void setGains(std::vector<double>& gains);
    void setGravity(double x, double y, double z);
    void setMaxIterations(std::size_t val);
    void setTree(const std::string& robot_model);
    void setRootAndTip(const std::string& chain_root, const std::string& chain_tip);


    void changeDynamicParams(const std::string& link, const double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia);
    void getResidualVector(std::vector<ResidualData>& sequence, std::vector<double>& residual_vec) const;


private:
    void iteration(const std::vector<ResidualData> &sequence,
                   const std::vector<Eigen::VectorXd>& model_coeff,
                   const std::vector<Eigen::VectorXd>& r_old,
                   const std::vector<Eigen::VectorXd>& vec_m,
                   std::vector<Eigen::VectorXd> &res) const;

    void integrate(const std::size_t iter,
                   const std::vector<ResidualData> &sequence,
                   const std::vector<Eigen::VectorXd> &model_coeff,
                   const std::vector<Eigen::VectorXd> &r_old,
                   Eigen::VectorXd &res) const;

    static void vector2EigenVector(const std::vector<double>& vec, Eigen::VectorXd& res);

private:
    mutable Jaco2KinematicsDynamicsModel model_;
    double accuracy_;
    std::size_t max_iter_;
    Eigen::MatrixXd gains_;
};

#endif // JACO2RESIDUALVECTOR_H