#ifndef JACO2RESIDUALVECTOR_H
#define JACO2RESIDUALVECTOR_H
#include <string>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
struct ResidualData
{
    double dt;
    double gx;
    double gy;
    double gz;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> torques;
};

/**
 * @brief The Jaco2ResidualVector class
 * Calculates external torques - Torques due to contacts !
 * Attention for integration update rate >> 1 Hz are required! Otherwise residual vector is numerically instable!
 */
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
    void getResidualVector(std::vector<ResidualData>& sequence, std::vector<Eigen::VectorXd>& residual_vec) const;
    void getResidualVector(std::vector<ResidualData>& sequence, std::vector<std::vector<double>>& residual_vec) const;
    void getResidualVector(const ResidualData &data,
                           const Eigen::VectorXd& last_residual,
                           const Eigen::VectorXd& last_integral,
                           Eigen::VectorXd& new_integral,
                           Eigen::VectorXd& new_residual);

    std::vector<double> getGravityTorque() const;

    int getNrOfJoints() const;
    int getAcceleration(const std::vector<double>& q,
                        const std::vector<double>& q_Dot,
                        const std::vector<double>& q_DotDot, std::vector<std::string> &links,
                        std::vector<KDL::Twist > &spatial_acc );

    std::vector<std::string> getLinkNames() const;

    static void vector2EigenVector(const std::vector<double>& vec, Eigen::VectorXd& res);
    static void eigenVector2vector(const Eigen::VectorXd &vec, std::vector<double> &res);

private:
    Eigen::VectorXd integration_step(const double dt, const Eigen::VectorXd& last_integral, const Eigen::VectorXd& next_integrant) const;


private:
    mutable Jaco2KinDynLib::Jaco2DynamicModel model_;
    Eigen::MatrixXd gains_;
    Eigen::VectorXd gravity_torque_;
};

#endif // JACO2RESIDUALVECTOR_H
