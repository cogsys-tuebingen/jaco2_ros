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
#include <kdl/chaindynparam.hpp>

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
    void setGravity(double x, double y, double z);
    void getGravity(double& gx, double& gy, double& gz);

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

    /**
     * @brief getAcceleration gets the accerlaration for all moving frames
     * @param gx acceleration of the basis frame x-component
     * @param gy acceleration of the basis frame y-component
     * @param gz acceleration of the basis frame z-component
     * @param q joint positions
     * @param q_Dot joint velocities
     * @param q_DotDot joint accelerations
     * @param links name of the link frame
     * @param spatial_acc the spatial velocity of the link in link coordinates
     * @return
     */
    int getAcceleration(const double gx, const double gy, const double gz,
                        const std::vector<double>& q,
                        const std::vector<double>& q_Dot,
                        const std::vector<double>& q_DotDot,
                        std::vector<std::string>& links,
                        std::vector<Eigen::Matrix<double, 6, 1> > &spatial_acc );
    /**
     * @brief getAcceleration gets the accerlaration for all moving frames
     * @param gx acceleration of the basis frame x-component
     * @param gy acceleration of the basis frame y-component
     * @param gz acceleration of the basis frame z-component
     * @param q joint positions
     * @param q_Dot joint velocities
     * @param q_DotDot joint accelerations
     * @param links name of the link frame
     * @param spatial_acc the spatial velocity of the link in link coordinates
     * @return
     */
    int getAcceleration(const double gx, const double gy, const double gz,
                        const std::vector<double>& q,
                        const std::vector<double>& q_Dot,
                        const std::vector<double>& q_DotDot, std::vector<std::string> &links,
                        std::vector<KDL::Twist > &spatial_acc );

    void changeDynamicParams(const std::string& link, const double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia);
    void changeDynamicParams(const std::string& link, const double mass, const Eigen::Vector3d& com, const Eigen::Matrix3d& inertia, double gx, double gy, double gz);

    void changeKineticParams(const std::string& link, const Eigen::Vector3d& trans, const Eigen::Matrix3d& rotation);
    /**
     * @brief getChainDynParam calculates the matrices H,C and G needed for the inverse dynamics problem. KDL wrapper
     * @param gx acceleration of the basis frame x-component
     * @param gy acceleration of the basis frame y-component
     * @param gz acceleration of the basis frame z-component
     * @param q joint positions
     * @param q_Dot joint velocities
     * @param q_DotDot joint accelerations
     * @param H interia matrix
     * @param C coriolis and centripetal forces
     * @param G influnce of gravitaional forces
     * Wrapper for KDL::chainDynParam:
     * Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) for the calculation torques out of the pose and derivatives. (inverse dynamics)
     * The algorithm implementation for H is based on the book "Rigid Body Dynamics Algorithms" of Roy Featherstone, 2008 (ISBN:978-0-387-74314-1) See page 107 for the pseudo-code. This algorithm is extended for the use of fixed joints
     * It calculates the joint-space inertia matrix, given the motion of the joints (q,qdot,qdotdot), external forces on the segments (expressed in the segments reference frame) and the dynamical parameters of the segments.
     */
    int getChainDynParam(const double gx, const double gy, const double gz,
                          const std::vector<double>& q,
                          const std::vector<double>& q_Dot,
                          Eigen::MatrixXd& H, Eigen::VectorXd& C, Eigen::VectorXd& G);

    void useUrdfDynamicParams();
    void getRandomConfig(std::vector<double>& config);
    /**
     * @brief getKDLSegmentIndex gets the KDL segment index for a given segment/ link name
     * @param name the name of the segment/ link
     * @return returns index of segment -1 if name == root_ ; -2 if link name not found.
     */
    int getKDLSegmentIndex(const std::string &name ) const;
    /**
     * @brief getKDLSegmentIndexFK gets the index of a link as needed for forward kinematics. root = 0, ...
     * @param name name of the segment/link.
     * @return the index of the segment/link.
     */
    int getKDLSegmentIndexFK(const std::string &name) const;
    int getNrOfJoints() const {return chain_.getNrOfJoints();}
    int getNrOfSegments() const {return chain_.getNrOfJoints();}
    double getLinkMass(const std::string &link) const;

    std::vector<std::string> getLinkNames() const;
    std::string getRootLink() const {return root_;}
    std::string getTipLink() const {return tip_;}
    /**
     * @brief getLinkCoM returns the displacemant from link center of mass to origin
     * @param link the name of the link
     * @return displacement vector form link origin(rotation axis) to the center of mass.
     */
    Eigen::Vector3d getLinkCoM(const std::string &link) const;
    /**
     * @brief getLinkInertia gets the inertia moment matrix of a link
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = rotation axis
     */
    Eigen::Matrix3d getLinkInertia(const std::string &link) const;

    /**
     * @brief getLinkInertiaCoM gets the inertia moment matrix of a link with respect to the center of mass;
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = center of mass (CoM)
     */
    Eigen::Matrix3d getLinkInertiaCoM(const std::string &link) const;

    Eigen::Vector3d getURDFLinkCoM(const std::string &link) const;
    Eigen::Matrix3d getURDFLinkInertiaCoM(const std::string &link) const;
    double getURDFLinkMass(const std::string &link) const;

    Eigen::Vector3d getLinkFixedTranslation(const std::string &link) const;
    Eigen::Matrix3d getLinkFixedRotation(const std::string &link) const;

    static void convert(const KDL::JntArray& in, std::vector<double>& out);
    static void convert(const std::vector<double>& in, KDL::JntArray& out);
    static void PoseTFToKDL(const tf::Pose& t, KDL::Frame& k);

    Eigen::MatrixXd getRigidBodyRegressionMatrix(const std::string& root, const std::string& tip,
                                                 const std::vector<double>& q,
                                                 const std::vector<double>& q_Dot,
                                                 const std::vector<double>& q_DotDot,
                                                 const double& gx,
                                                 const double& gy,
                                                 const double& gz,
                                                 const bool project = true);

    void modifiedRNE(const std::string& root, const std::string& tip,
                    const double gx,
                    const double gy,
                    const double gz,
                    const Eigen::VectorXd& q1,
                    const Eigen::VectorXd& q2,
                    const Eigen::VectorXd& q3,
                    const Eigen::VectorXd& q4,
                    Eigen::VectorXd& res);

    static Eigen::Matrix3d skewSymMat(const KDL::Vector& vec);
    static Eigen::Matrix<double, 3, 6> inertiaProductMat(const KDL::Vector& vec);
    static Eigen::Matrix<double, 6, 6> kdlFrame2Spatial(const KDL::Frame& frame);
    static Eigen::Matrix<double, 3, 3> kdlMatrix2Eigen(const KDL::Rotation& rot);
    static void kdlJntArray2Eigen(const KDL::JntArray& q, Eigen::VectorXd &res);
    static void kdlMatrix2Eigen(const KDL::JntSpaceInertiaMatrix& mat, Eigen::MatrixXd &res);

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
    KDL::JntArray lowerLimits_;
    KDL::JntArray upperLimits_;

    void initialize();

    /**
     * @brief getLinkInertiaCoM gets the inertia moment matrix of a link with respect to the center of mass;
     * @param link the name of the link
     * @return the inertia moment matrix at link origin = center of mass (CoM)
     */
    KDL::RotationalInertia getLinkInertiaCoM(const int id) const;



};

#endif // JACO2KINEMATICSDYNAMICS_H
