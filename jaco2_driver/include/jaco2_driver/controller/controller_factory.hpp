#ifndef CONTROLLER_FACTORY_HPP
#define CONTROLLER_FACTORY_HPP

#include <jaco2_driver/controller/velocity_controller.h>
#include <jaco2_driver/controller/collision_repelling_velocity_controller.hpp>
#include <jaco2_driver/controller/trajectory_tracking_controller.h>
#include <jaco2_driver/controller/collision_repelling_p2p_controller.h>
#include <jaco2_driver/controller/collision_repelling_velocity_controller.hpp>
#include <jaco2_driver/controller/torque_trajectory_controller.h>
#include <jaco2_driver/controller/collision_repelling_p2p_torque_controller.h>

namespace ControllerFactory
{
    const std::string velocity_controller = "VEL";
    const std::string velocity_collision_controller = "VEL_COLL";

    const std::string trajectory_p2p_velocity_controller = "TRAJ_P2P_VEL";
    const std::string trajectory_p2p_velocity_collision_controller = "TRAJ_P2P_VEL_COLL";
    const std::string trajectory_p2p_torque_controller = "TRAJ_P2P_TOR";
    const std::string trajectory_p2p_torque_collision_controller = "TRAJ_P2P_TOR_COLL";

    VelocityController makeVelocityController(Jaco2State& state, Jaco2API& api, std::string type)
    {
        if(type == velocity_controller){
            return VelocityController(state, api);
        }
        if(type == velocity_collision_controller){
            return CollisionRepellingVelocityController(state, api);
        }
        return VelocityController(state, api);
    }

    TrajectoryTrackingController makeTrajectoryTrackingController(Jaco2State& state, Jaco2API& api, std::string type)
    {
        if(type == trajectory_p2p_velocity_controller){
            return Point2PointVelocityController(state, api);
        }
        if(type == trajectory_p2p_velocity_collision_controller){
            return CollisionReplellingP2PController(state, api);
        }
        if(type == trajectory_p2p_torque_controller){
            return TorqueTrajectoryController(state, api);
        }
        if(type == trajectory_p2p_torque_collision_controller){
            return CollisionReplellingP2PTorqueController(state, api);
        }
    }
}
#endif // CONTROLLER_FACTORY_HPP
