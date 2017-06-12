#ifndef COLLISION_REPELLING_P2P_CONTROLLER_H
#define COLLISION_REPELLING_P2P_CONTROLLER_H
#include "data_conversion.h"
#include <jaco2_driver/point_2_point_velocity_controller.h>
#include <jaco2_driver/torque_controller.h>
#include <jaco2_driver/torque_trajectory_controller.h>
#include <jaco2_kin_dyn_lib/jaco2_residual_vector.h>
#include <jaco2_kin_dyn_lib/joint_vel_pos_estimator.h>

class CollisionReplellingP2PController : public Jaco2Controller
{
public:
    CollisionReplellingP2PController(Jaco2State &state, Jaco2API &api);

    virtual void write() override;
    virtual void start() override;
    virtual bool isDone() const override;

    void setThreshold(double threshold);
    void setRobotModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    void setReflexGain(const AngularInfo& kr);
    void setCorrectionGains(const AngularInfo& kp, const AngularInfo kd);
    void setVelocityControlGains(double p, double i, double d);

    //Velocity Controller Gains
    void setTrajectory(const JointTrajectory& trajectory);

    //Tracking Gains
    void setGainP(const ManipulatorInfo &gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);
    AngularInfo getJointError() const ;

private:
    void reflex();
    double getResiduals();
    void estimateGravity(double& gx, double &gy, double& gz);
    void getResidualsData(ResidualData& data);
    void resetResiduals();
//    void calculateEsum(const AngularInfo& diff);


private:
    VelocityController reflex_controller_;
    Point2PointVelocityController tracking_controller_;
    double threshold_;
    bool first_coll_;
    bool in_collision_;
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
    std::deque<AngularInfo> e_buffer_;
//    AngularInfo esum_;
    double kp_;
    double ki_;


};

#endif // COLLISION_REPELLING_P2P_CONTROLLER_H
