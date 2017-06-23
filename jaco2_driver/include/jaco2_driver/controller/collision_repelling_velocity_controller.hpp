#ifndef COLLISION_REPELLING_VELOCITY_CONTROLLER_HPP
#define COLLISION_REPELLING_VELOCITY_CONTROLLER_HPP
#include "velocity_controller.h"
#include "collision_reaction.h"

class CollisionRepellingVelocityController : public VelocityController
{
public:
    CollisionRepellingVelocityController(Jaco2State &state, Jaco2API &api)
        : VelocityController(state, api),
          collision_reaction_(state)
    {
        collision_reaction_.setRobotModel("/robot_description", "jaco_link_base", "jaco_link_hand");
        last_cmd_rep_  = std::chrono::high_resolution_clock::now();
    }

    void setVelocity(const TrajectoryPoint& tp)
    {
        if(!collision_reaction_.inCollision()){
            collision_reaction_.resetResiduals();
            VelocityController::setVelocity(tp);
            last_cmd_rep_  = std::chrono::high_resolution_clock::now();
        }
    }

    virtual void setConfig(jaco2_driver::jaco2_driver_configureConfig& cfg) override
    {
        VelocityController::setConfig(cfg);
        collision_reaction_.setConfig(cfg);
    }

    virtual void write() override
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto durationLast = now - last_cmd_rep_;
        last_cmd_rep_ = now;
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
        collision_reaction_.update(dt);

        double residual = collision_reaction_.getResidualsNorm();

        if(collision_reaction_.inCollision() && desired_.Position.HandMode == HAND_NOMOVEMENT ){
            while(collision_reaction_.inCollision()){
                ROS_INFO_STREAM("Repelling! collision detected: "<< residual);
                auto cmd = collision_reaction_.velocityControlReflex();
                VelocityController::setVelocity(cmd);
                VelocityController::write();
                usleep(5000);
                state_.read();
                now = std::chrono::high_resolution_clock::now();
                last_cmd_rep_ = now;
                dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
                //        std::cout << dt << std::endl;
                collision_reaction_.update(dt);
            }

        }
        else{

            VelocityController::write();
        }

    }

    void setThreshold(double threshold)
    {
        collision_reaction_.setThreshold(threshold);
    }

    void setRobotModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip)
    {
        collision_reaction_.setRobotModel(robot_model, chain_root, chain_tip);
    }

    void setReflexGain(const AngularInfo& kr)
    {
        collision_reaction_.setReflexGain(kr);
    }



private:

    CollisionReaction collision_reaction_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_cmd_rep_;
};
#endif // COLLISION_REPELLING_VELOCITY_CONTROLLER_HPP
