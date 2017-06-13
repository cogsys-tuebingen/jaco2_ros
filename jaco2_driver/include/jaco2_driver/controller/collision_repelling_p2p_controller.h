#ifndef COLLISION_REPELLING_P2P_CONTROLLER_H
#define COLLISION_REPELLING_P2P_CONTROLLER_H
#include <jaco2_driver/data_conversion.h>
#include <jaco2_driver/controller/point_2_point_velocity_controller.h>
#include <jaco2_driver/controller/velocity_controller.h>
#include <jaco2_driver/controller/collision_reaction.h>

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
    void setVelocityControlGains(double p, double i, double d);

    void setTrajectory(const JointTrajectory& trajectory);

    //Tracking Gains
    void setGainP(const ManipulatorInfo &gains);
    void setGainI(const ManipulatorInfo &gains);
    void setGainD(const ManipulatorInfo &gains);
    AngularInfo getJointError() const ;



private:
    VelocityController reflex_controller_;
    Point2PointVelocityController tracking_controller_;
    CollisionReaction collision_reaction_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_cmd_rep_;



};

#endif // COLLISION_REPELLING_P2P_CONTROLLER_H
