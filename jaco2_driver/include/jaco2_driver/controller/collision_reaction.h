#ifndef COLLISION_REACTION_H
#define COLLISION_REACTION_H

#include <kinova/KinovaTypes.h>
#include <jaco2_driver/jaco2_state.h>
#include <jaco2_kin_dyn_lib/jaco2_residual_vector.h>
/**
 * @brief The CollisionReaction class
 *
 * Example Usage:
 *
 * if(!inCollision){
 *    NORMAL OPERATION MODE
 * }
 * else if(inCollision() && !energyDisipation(){
 *    auto cmd = torqueControlReflex()
 *    execute torque controll cmd
 * }
 * else if(energyDisipation()){
 *    torqueControlEnergyDisspation()
 *    execute cmd
 * }
 *
 * De Luca, A., Albu-Schäffer, A., Haddadin, S., & Hirzinger, G. (2006).
 * Collision detection and safe reaction with the DLR-III lightweight manipulator arm.
 * IEEE International Conference on Intelligent Robots and Systems, 1623–1630. https://doi.org/10.1109/IROS.2006.282053
 */
class CollisionReaction
{
public:
    CollisionReaction(Jaco2State& state);

    void start();
    void setThreshold(double threshold);
    void setRobotModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    void setReflexGain(const AngularInfo& kr);

    TrajectoryPoint velocityControlReflex();
    TrajectoryPoint velocityEnergyDisspation();
    AngularInfo torqueControlReflex();
    AngularInfo torqueControlEnergyDisspation();
    AngularInfo getResiduals() const;
    double getResidualsNorm() const;

    void estimateGravity(double& gx, double &gy, double& gz);
    void resetResiduals();

    void update(double dt);

    bool inCollision() const;
    bool energyDisipation() const;
    bool firstCollision() const;
    std::size_t getCollisionCounter() const;


private:
    void getResidualsData(ResidualData &data);
    void updateResiduals();
    double energyDisipation(AngularInfo &vel, AngularInfo &lower, AngularInfo &upper, std::size_t id) const;
    TrajectoryPoint calculateVelocity(AngularInfo& cmd);

private:
    Jaco2State &state_;
    bool in_collision_;
    std::size_t collision_counter_;
    double threshold_;
    double stop_threshold_;
    double dt_;
    double residualNorm_;
    Jaco2ResidualVector resiudals_;
    Eigen::VectorXd last_integral_;
    Eigen::VectorXd last_residual_;
    std::deque<Eigen::Vector3d> filter_g_;

    AngularInfo kr_;
    AngularInfo vel_bound_;
    AngularInfo max_torques_;
    AngularInfo velocity_threshold_;
};
#endif // COLLISION_REACTION_H
