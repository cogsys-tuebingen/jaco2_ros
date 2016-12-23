#ifndef JACO2KINEMATICMODEL_H
#define JACO2KINEMATICMODEL_H
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
#include <kdl/solveri.hpp>

class Jaco2KinematicModel
{
public:
    Jaco2KinematicModel();
    Jaco2KinematicModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);


    void setTree(const std::string& robot_model);
    void setRootAndTip(const std::string& chain_root, const std::string& chain_tip);
    void setGravity(double x, double y, double z);
    void getGravity(double& gx, double& gy, double& gz);

    //Solver Calls

    /**
     * @brief getFKPose Given the joint variables q_in the world space coordinates are calculated. KDL wrapper
     * @param q_in  input: joint variables
     * @param out   output: pose (position and orientation)
     * @param link  input: the link for which the pose should be calculated.
     * @return
     */
    int getFKPose(const std::vector<double>& q_in, tf::Pose& out, const std::string link);

    /**
     * @brief getFKPose Given the joint variables q_in the world space coordinates are calculated. KDL wrapper
     * @param q_in  input: joint variables
     * @param out   output: pose (position and orientation)
     * @param link  input: the link for which the pose should be calculated.
     * @return
     */
    int getFKPose(const std::vector<double>& q_in, KDL::Frame& out, const std::string link);

    /**
     * @brief getIKSolution given a end effector pose a corresponding joint configuration is search. trac_ik wrapper
     * @param pose      input: the pose (position, orientation) of the end-effector
     * @param result    output: the joint variables.
     * @param seed      optional input: seed for joint value search. if not given a random seed is choosen.
     * @return
     */
    int getIKSolution(const tf::Pose &pose, std::vector<double> &result, const std::vector<double>& seed = std::vector<double>());

    void changeKineticParams(const std::string& link, const Eigen::Vector3d& trans, const Eigen::Matrix3d& rotation);


    void getRandomConfig(std::vector<double>& config);
    /**
     * @brief getKDLSegmentIndex gets the KDL segment index for a given segment/ link name
     * @param name the name of the segment/ link
     * @return returns index of segment -1 if name == root_ ; -2 if link name not found.
     */
    void getRandomConfig(Eigen::VectorXd& config);
    void useUrdfDynamicParams();
    int getKDLSegmentIndex(const std::string &name ) const;
    /**
     * @brief getKDLSegmentIndexFK gets the index of a link as needed for forward kinematics. root = 0, ...
     * @param name name of the segment/link.
     * @return the index of the segment/link.
     */
    int getKDLSegmentIndexFK(const std::string &name) const;
    int getNrOfJoints() const {return chain_.getNrOfJoints();}
    int getNrOfSegments() const {return chain_.getNrOfJoints();}

    std::vector<std::string> getLinkNames() const;
    std::string getRootLink() const {return root_;}
    std::string getTipLink() const {return tip_;}


    Eigen::Vector3d getLinkFixedTranslation(const std::string &link) const;
    Eigen::Matrix3d getLinkFixedRotation(const std::string &link) const;

    static void convert(const KDL::JntArray& in, std::vector<double>& out);
    static void convert(const std::vector<double>& in, KDL::JntArray& out);
    static void PoseTFToKDL(const tf::Pose& t, KDL::Frame& k);



    /**
     * @brief getRotationAxis gets the rotation axis of the link in the link frame
     * @param link the name of the link
     * @param rot_axis the rotation axis normaly z-axis (DH connvention)
     */
    void getRotationAxis(const std::string &link, KDL::Vector &rot_axis);
    void getRotationAxis(const std::string &link, Eigen::Vector3d& rot_axis);


    static Eigen::Matrix3d skewSymMat(const KDL::Vector& vec);

protected:
    std::string urdf_param_;
    std::string root_;
    std::string tip_;
    urdf::Model robot_model_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    KDL::Chain chainFile_;
    KDL::Vector gravity_;

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> solverFK_;
    std::shared_ptr<TRAC_IK::TRAC_IK> solverIK_;
    std::vector<std::uniform_real_distribution<double> > jointDist_;
    std::default_random_engine randEng_;
    KDL::JntArray lowerLimits_;
    KDL::JntArray upperLimits_;

    void initialize();



};

#endif // JACO2KINEMATICMODEL_H