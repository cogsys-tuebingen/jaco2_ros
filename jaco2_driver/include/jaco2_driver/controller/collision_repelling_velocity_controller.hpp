#ifndef COLLISION_REPELLING_VELOCITY_CONTROLLER_HPP
#define COLLISION_REPELLING_VELOCITY_CONTROLLER_HPP
#include "velocity_controller.h"
#include "collision_reaction.h"

class CollisionRepellingVelocityController : public VelocityController
{
public:
    CollisionRepellingVelocityController(Jaco2State &state, Jaco2API &api)
        : VelocityController(state, api),
          set_model_(false),
          collision_reaction_(state)
    {
        last_cmd_rep_  = std::chrono::high_resolution_clock::now();
    }

    void setVelocity(const TrajectoryPoint& tp) override
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
        set_model_ = true;
    }

    virtual void write() override
    {
        if(!set_model_){
            return;
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto durationLast = now - last_cmd_rep_;
        last_cmd_rep_ = now;
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
        collision_reaction_.update(dt);

        double residual = collision_reaction_.getResidualsNorm();

        if(collision_reaction_.inCollision() && desired_.Position.HandMode == HAND_NOMOVEMENT ){
            while(collision_reaction_.inCollision()){
                ROS_WARN_STREAM("Repelling! collision detected: "<< residual);
                auto cmd = collision_reaction_.velocityControlReflex();
//                for(int i = 0; i < 2; ++i){
                    VelocityController::setVelocity(cmd);
                    VelocityController::write();
                    usleep(5000);
//                }
                state_.read();

                now = std::chrono::high_resolution_clock::now();
                last_cmd_rep_ = now;
                dt = std::chrono::duration_cast<std::chrono::microseconds>(durationLast).count()*1e-6;
                //        std::cout << dt << std::endl;
                collision_reaction_.update(dt);
                residual = collision_reaction_.getResidualsNorm();
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
        set_model_ = true;
    }

    void setReflexGain(const AngularInfo& kr)
    {
        collision_reaction_.setReflexGain(kr);
    }



private:
    bool set_model_;
    CollisionReaction collision_reaction_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_cmd_rep_;
};
#endif // COLLISION_REPELLING_VELOCITY_CONTROLLER_HPP
