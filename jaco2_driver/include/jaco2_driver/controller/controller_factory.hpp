#ifndef CONTROLLER_FACTORY_HPP
#define CONTROLLER_FACTORY_HPP

#include <jaco2_driver/jaco2_driver_constants.h>
#include <jaco2_driver/controller/velocity_controller.h>
#include <jaco2_driver/controller/collision_repelling_velocity_controller.hpp>
#include <jaco2_driver/controller/trajectory_tracking_controller.h>
#include <jaco2_driver/controller/collision_repelling_p2p_controller.h>
#include <jaco2_driver/controller/collision_repelling_velocity_controller.hpp>
#include <jaco2_driver/controller/torque_trajectory_controller.h>
#include <jaco2_driver/controller/collision_repelling_p2p_torque_controller.h>
/**
 * Spwan different controllers
 */
namespace ControllerFactory
{
std::shared_ptr<VelocityController> makeVelocityController(Jaco2State& state, Jaco2API& api, Jaco2Controller::TerminationCallback& t, const std::string& type)
{
    if(type == Jaco2DriverConstants::velocity_controller){
        return std::make_shared<VelocityController>(VelocityController(state, api, t));
    }
    else if(type == Jaco2DriverConstants::velocity_collision_controller){
        return std::make_shared<CollisionRepellingVelocityController>(CollisionRepellingVelocityController(state, api, t));
    }
    else{
        return nullptr;
    }
}

std::shared_ptr<TrajectoryTrackingController> makeTrajectoryTrackingController(Jaco2State& state, Jaco2API& api,
                                                                               Jaco2Controller::TerminationCallback& t,
                                                                               const std::string& type)
{
    if(type == Jaco2DriverConstants::trajectory_p2p_velocity_controller){
        return std::make_shared<Point2PointVelocityController>(Point2PointVelocityController(state, api, t));
    }
    else if(type == Jaco2DriverConstants::trajectory_p2p_velocity_collision_controller){
        return std::make_shared<CollisionReplellingP2PController>(CollisionReplellingP2PController(state, api, t));
    }
    else if(type == Jaco2DriverConstants::trajectory_p2p_torque_controller){
        return std::make_shared<TorqueTrajectoryController>(TorqueTrajectoryController(state, api, t));
    }
    else if(type == Jaco2DriverConstants::trajectory_p2p_torque_collision_controller){
        return std::make_shared<CollisionReplellingP2PTorqueController>(CollisionReplellingP2PTorqueController(state, api, t));
    }
    else{
        return nullptr;
    }

}
}
#endif // CONTROLLER_FACTORY_HPP
