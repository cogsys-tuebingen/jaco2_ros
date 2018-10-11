#ifndef COLLISION_REPELLING_P2P_TORQUE_CONTROLLER_H
#define COLLISION_REPELLING_P2P_TORQUE_CONTROLLER_H
#include <jaco2_driver/data_conversion.h>
#include <jaco2_driver/controller/trajectory_tracking_controller.h>
#include <jaco2_driver/controller/torque_controller.h>
#include <jaco2_driver/controller/torque_trajectory_controller.h>
#include <jaco2_driver/controller/collision_reaction.h>

/**
 * @brief The CollisionReplellingP2PTorqueController class
 * Still in an experimental state...
 */
class CollisionReplellingP2PTorqueController : public TrajectoryTrackingController
{
public:
    CollisionReplellingP2PTorqueController(Jaco2State &state, Jaco2API &api, TerminationCallback& t);


    virtual void write() override;
    virtual void start() override;
    virtual void stop() override;
    virtual bool isDone() const override;

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override;
    void setThreshold(double threshold);
    void setRobotModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    void setReflexGain(const AngularInfo& kr);
    void setCorrectionGains(const AngularInfo& kp, const AngularInfo kd);
    void setVelocityControlGains(double p, double i, double d);

    void setTrajectory(const JointTrajectory& trajectory);
    //Tracking Gains
    void setGainP(const ManipulatorInfo &gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);
    AngularInfo getJointError() const ;



private:
    TorqueController reflex_controller_;
    TorqueTrajectoryController tracking_controller_;
    CollisionReaction collision_reaction_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_cmd_rep_;

};

#endif // COLLISION_REPELLING_P2P_TORQUE_CONTROLLER_H
