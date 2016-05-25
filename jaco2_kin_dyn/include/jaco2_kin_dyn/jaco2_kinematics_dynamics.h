#ifndef JACO2KINEMATICSDYNAMICS_H
#define JACO2KINEMATICSDYNAMICS_H
//System
#include <string>
#include <vector>
#include <memory>
//ROS
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <tf/tf.h>
// trac_ik
#include <trac_ik/trac_ik.hpp>
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

class Jaco2KinematicsDynamicsModel
{
public:
    Jaco2KinematicsDynamicsModel();
    Jaco2KinematicsDynamicsModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);


    void setTree(const std::string& robot_model);
    void setRootAndTip(const std::string& chain_root, const std::string& chain_tip);

    //Solver Calls
    /**
     * @brief getTorques Inverse dynamics solver call. Wrapper for KDL Recursive Newton Euler Algorithem cf. orocos project and "Rigid Body Dynamics Algorithms" of Roy Featherstone, 2008 (ISBN:978-0-387-74314-1) See page 96 for the pseudo-code.
     * @param q         input: the joint variables, positions
     * @param q_Dot     input: joint velocities
     * @param q_DotDot  input: joint accelerations
     * @param torques   output: joint torques
     * @param wrenches_ext optional input: external forces (not gravity)
     * @return 0 if successful
     */
    int getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                   std::vector<double>& torques, const std::vector<Wrench>& wrenches_ext = std::vector<Wrench>());

    /**
     * @brief getFKPose Given the joint variables q_in the world space coordinates are calculated. KDL wrapper
     * @param q_in  input: joint variables
     * @param out   output: pose (position and orientation)
     * @param link  input: the link for which the pose should be calculated.
     * @return
     */
    int getFKPose(const std::vector<double>& q_in, tf::Pose& out, std::string link);

    /**
     * @brief getIKSolution given a end effector pose a corresponding joint configuration is search. trac_ik wrapper
     * @param pose      input: the pose (position, orientation) of the end-effector
     * @param result    output: the joint variables.
     * @param seed      optional input: seed for joint value search. if not given a random seed is choosen.
     * @return
     */
    int getIKSolution(const tf::Pose &pose, std::vector<double> &result, const std::vector<double>& seed = std::vector<double>());

    void changeDynamicParams(const std::string& link, const double mass, const tf::Vector3& com, const tf::Matrix3x3 inertia);

    void changeKineticParams(const std::string& link, const tf::Vector3& trans, const tf::Matrix3x3 rotation);

    void useUrdfDynamicParams(){chain_ = chainFile_;}
    void getRandomConfig(std::vector<double>& config);
    int getKDLSegmentIndex(const std::string &name ) const;
    int getKDLSegmentIndexFK(const std::string &name) const;
    int getNrOfJoints() const {return chain_.getNrOfJoints();}
    int getNrOfSegments() const {return chain_.getNrOfJoints();}
    double getLinkMass(const std::string &link) const;
    std::string getRootLink() const {return root_;}
    std::string getTipLink() const {return tip_;}
    /**
     * @brief getLinkCoM returns the displacemant from link center of mass to origin
     * @param link the name of the link
     * @return displacement vector form link origin(rotation axis) to the center of mass.
     */
    tf::Vector3 getLinkCoM(const std::string &link) const;
    /**
     * @brief getLinkInertia gets the inertia moment matrix of a link
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = rotation axis
     */
    tf::Matrix3x3 getLinkInertia(const std::string &link) const;

    tf::Vector3 getLinkFixedTranslation(const std::string &link) const;
    tf::Matrix3x3 getLinkFixedRotation(const std::string &link) const;

    static void convert(const KDL::JntArray& in, std::vector<double>& out);
    static void convert(const std::vector<double>& in, KDL::JntArray& out);
    static void PoseTFToKDL(const tf::Pose& t, KDL::Frame& k);


private:
    std::string urdf_param_;
    std::string root_;
    std::string tip_;
    urdf::Model robot_model_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::Chain chainFile_;
    KDL::Vector gravity_;
    std::shared_ptr<KDL::ChainIdSolver_RNE> solverID_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> solverFK_;
    std::shared_ptr<TRAC_IK::TRAC_IK> solverIK_;
    std::vector<std::uniform_real_distribution<double> > jointDist_;
    std::default_random_engine randEng_;

    void initialize();

};

#endif // JACO2KINEMATICSDYNAMICS_H
