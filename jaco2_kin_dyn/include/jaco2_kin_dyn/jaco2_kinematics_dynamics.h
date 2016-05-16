#ifndef JACO2KINEMATICSDYNAMICS_H
#define JACO2KINEMATICSDYNAMICS_H
//System
#include <string>
//ROS
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
//Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/solveri.hpp>

class Jaco2KinematicsDynamics
{
public:
    Jaco2KinematicsDynamics();
    Jaco2KinematicsDynamics(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    Jaco2KinematicsDynamics(const urdf::Model& robot_model, const std::string& chain_root, const std::string& chain_tip);

    void setTree(const std::string& robot_model);
    void setTree(const urdf::Model& robot_model);
    void setRootAndTip(const std::string& chain_root, const std::string& chain_tip);

private:
    std::string root_;
    std::string tip_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::SolverI solver_;
};

#endif // JACO2KINEMATICSDYNAMICS_H
