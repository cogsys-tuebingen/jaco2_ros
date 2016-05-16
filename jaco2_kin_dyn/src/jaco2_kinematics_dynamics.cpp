#include "../include/jaco2_kin_dyn/jaco2_kinematics_dynamics.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

Jaco2KinematicsDynamics::Jaco2KinematicsDynamics()
{
}

Jaco2KinematicsDynamics::Jaco2KinematicsDynamics(const std::string &robot_model, const std::string& chain_root, const std::string& chain_tip):
    root_(chain_root), tip_(chain_tip)
{
    if (!kdl_parser::treeFromString(robot_model, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    //    std::string end_segment_name = tree_.getSegments().end()->second.segment.getName(); //better: use input
    //    std::string root_segment_name = tree_.getSegments().begin()->second.segment.getName(); // better: use input
    if(tree_.getChain(root_,tip_,chain_)){

    }
    //    solver_ = KDL::ChainIdSolver_RNE()
}

Jaco2KinematicsDynamics::Jaco2KinematicsDynamics(const urdf::Model &robot_model, const std::string &chain_root, const std::string &chain_tip):
    root_(chain_root), tip_(chain_tip)
{
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
    }

}

void Jaco2KinematicsDynamics::setTree(const std::string &robot_model)
{
    if (!kdl_parser::treeFromString(robot_model, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
    }
}

void Jaco2KinematicsDynamics::setTree(const urdf::Model &robot_model)
{
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
    }
}

void::Jaco2KinematicsDynamics::setRootAndTip(const std::string &chain_root, const std::string &chain_tip)
{
    root_ = chain_root;
    tip_ = chain_tip;
}
