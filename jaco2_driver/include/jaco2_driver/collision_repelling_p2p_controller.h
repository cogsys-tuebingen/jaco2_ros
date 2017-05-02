#ifndef COLLISION_REPELLING_P2P_CONTROLLER_H
#define COLLISION_REPELLING_P2P_CONTROLLER_H
#include <jaco2_driver/point_2_point_velocity_controller.h>
#include <jaco2_kin_dyn_lib/jaco2_residual_vector.h>

class CollisionReplellingP2PController : public Point2PointVelocityController
{
    CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api);

    virtual void write() override;

    virtual bool isDone() const;

    void setThreshold(double threshold);
    void setRobotModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

private:
    double getResiduals();
    void estimateGravity(double& gx, double &gy, double& gz);
    void getResidualsData(ResidualData& data);


private:
    double threshold_;
    Jaco2ResidualVector resiudals_;
    Eigen::VectorXd last_integral_;
    Eigen::VectorXd last_residual_;
    std::deque<Eigen::Vector3d> filter_g_;

};

#endif // COLLISION_REPELLING_P2P_CONTROLLER_H
