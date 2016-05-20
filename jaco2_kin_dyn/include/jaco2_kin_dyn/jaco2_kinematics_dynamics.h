#ifndef JACO2KINEMATICSDYNAMICS_H
#define JACO2KINEMATICSDYNAMICS_H
//System
#include <string>
#include <vector>
//ROS
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf/tf.h>
//Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/solveri.hpp>

struct Wrench{

    double force[3];
    double torque[3];

    Wrench()
    {
        force[0] = 0;
        force[1] = 0;
        force[2] = 0;
        torque[0] = 0;
        torque[1] = 0;
        torque[2] = 0;
    }

    Wrench(double fx, double fy, double fz,
           double tx, double ty, double tz)
    {
        force[0] = fx;
        force[1] = fy;
        force[2] = fz;
        torque[0] = tx;
        torque[1] = ty;
        torque[2] = tz;
    }

    KDL::Wrench toKDL() const
    {
        KDL::Vector f(force[0],force[2],force[3]);
        KDL::Vector tau(torque[0],torque[1],torque[3]);

        return KDL::Wrench(f,tau);
    }
};

class Jaco2KinematicsDynamics
{
public:
    Jaco2KinematicsDynamics();
    Jaco2KinematicsDynamics(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
    Jaco2KinematicsDynamics(const urdf::Model& robot_model, const std::string& chain_root, const std::string& chain_tip);

    void setTree(const std::string& robot_model);
    void setTree(const urdf::Model& robot_model);
    void setRootAndTip(const std::string& chain_root, const std::string& chain_tip);

    int getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                   std::vector<double>& torques, const std::vector<Wrench>& wrenches_ext = std::vector<Wrench>());

    int getFKPose(const std::vector<double>& q_in, tf::Pose& out, std::string link);

    int getKDLSegmentIndexFK(const std::string &name) const;

    int getNrOfJoints() const {return chain_.getNrOfJoints();}
    int getNrOfSegments() const {return chain_.getNrOfJoints();}
    std::string getRootLink() const {return root_;}
    std::string getTipLink() const {return tip_;}

    static void convert(const KDL::JntArray& in, std::vector<double>& out);
    static void convert(const std::vector<double>& in, KDL::JntArray& out);


private:
    std::string root_;
    std::string tip_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::Vector gravity_;
    KDL::ChainIdSolver_RNE solverID_;
};

#endif // JACO2KINEMATICSDYNAMICS_H