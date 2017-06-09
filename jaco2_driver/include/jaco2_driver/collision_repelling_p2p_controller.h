#ifndef COLLISION_REPELLING_P2P_CONTROLLER_H
#define COLLISION_REPELLING_P2P_CONTROLLER_H
#include "data_conversion.h"
#include <jaco2_driver/point_2_point_velocity_controller.h>
#include <jaco2_kin_dyn_lib/jaco2_residual_vector.h>
#include <jaco2_kin_dyn_lib/joint_vel_pos_estimator.h>

class CollisionReplellingP2PController : public Point2PointVelocityController
{
public:
    CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api);

    virtual void write() override;

    virtual bool isDone() const;

    void setThreshold(double threshold);
    void setRobotModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    void setReflexGain(const AngularInfo& kr);
    void setCorrectionGains(const AngularInfo& kp, const AngularInfo kd);

private:
    void reflex();
    double getResiduals();
    void estimateGravity(double& gx, double &gy, double& gz);
    void getResidualsData(ResidualData& data);


private:
    double threshold_;
    bool first_coll_;
    Jaco2ResidualVector resiudals_;
    Jaco2KinDynLib::JointVelPosEstimator estimator_;
    Eigen::VectorXd last_integral_;
    Eigen::VectorXd last_residual_;
    std::deque<Eigen::Vector3d> filter_g_;
    double dt_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_cmd_rep_;
    AngularInfo kr_;
    AngularInfo kpq_;
    AngularInfo kdq_;



};

#endif // COLLISION_REPELLING_P2P_CONTROLLER_H
