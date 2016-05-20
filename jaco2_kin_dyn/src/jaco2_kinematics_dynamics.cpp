#include <jaco2_kin_dyn/jaco2_kinematics_dynamics.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>


Jaco2KinematicsDynamics::Jaco2KinematicsDynamics():
    gravity_(0,0,9.81), solverID_(chain_,gravity_)
{
}

Jaco2KinematicsDynamics::Jaco2KinematicsDynamics(const std::string &robot_model, const std::string& chain_root, const std::string& chain_tip):
    root_(chain_root), tip_(chain_tip), gravity_(0,0,9.81), solverID_(chain_,gravity_)
{
    setTree(robot_model);
//    KDL::Segment seg2 = chain_.getSegment(2);
//    KDL::RigidBodyInertia I2 = seg2.getInertia();
//    std::cout <<"Mass " << I2.getMass() << std::endl;
//    std::cout <<I2.getRotationalInertia().data[0] << std::endl;
//    KDL::Vector com = seg2.getInertia().getCOG();
//    std::cout << com(0) << ", " << com(1) << ", " << com(2) << std::endl;
//    std::cout << "Number of Joints: " << chain_.getNrOfJoints() << " | Number of Segments: " << chain_.getNrOfSegments() << std::endl;
}

Jaco2KinematicsDynamics::Jaco2KinematicsDynamics(const urdf::Model &robot_model, const std::string &chain_root, const std::string &chain_tip):
    root_(chain_root), tip_(chain_tip), gravity_(0,0,9.81), solverID_(chain_,gravity_)
{
    setTree(robot_model);
}

void Jaco2KinematicsDynamics::setTree(const std::string &robot_model)
{
    if (!kdl_parser::treeFromString(robot_model, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    if(tree_.getChain(root_,tip_,chain_)){

        solverID_ =  KDL::ChainIdSolver_RNE(chain_,gravity_);
        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain_);
    }
    else{
        ROS_ERROR("Chain extraction is not possible. Solver is not probably initalized");
    }
}

void Jaco2KinematicsDynamics::setTree(const urdf::Model &robot_model)
{
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    if(tree_.getChain(root_,tip_,chain_)){

        solverID_ = KDL::ChainIdSolver_RNE(chain_,gravity_);

    }
    else{
        ROS_ERROR("Chain extraction is not possible. Solver is not probably initalized");
    }
}

void::Jaco2KinematicsDynamics::setRootAndTip(const std::string &chain_root, const std::string &chain_tip)
{
    root_ = chain_root;
    tip_ = chain_tip;
}

int Jaco2KinematicsDynamics::getTorques(const std::vector<double> &q, const std::vector<double> &q_Dot, const std::vector<double> &q_DotDot,
                                        std::vector<double> &torques, const std::vector<Wrench> &wrenches_ext)
{
    if(q.size() != q_Dot.size() && q.size() != q_DotDot.size() && q.size() != chain_.getNrOfJoints()){
        ROS_ERROR_STREAM("Dimension mismatch of input: Dimenion of q is" << q.size() << ". While q_Dot has dimension " << q_Dot.size() << " and q_DotDot "
                         << q_DotDot.size()<<". The KDL chain contains " <<  chain_.getNrOfJoints() << "  joints." << std::endl);
        return KDL::SolverI::E_UNDEFINED;
    }

    KDL::JntArray qkdl(q.size());
    KDL::JntArray qkdl_Dot(q.size());
    KDL::JntArray qkdl_DotDot(q.size());
    KDL::JntArray torques_kdl(chain_.getNrOfJoints());
    KDL::Wrenches wrenches;
    wrenches.resize(chain_.getNrOfSegments());
    for(std::size_t i = 0; i < q.size(); ++i) {
        qkdl(i) = q[i];
        qkdl_Dot(i) = q_Dot[i];
        qkdl_DotDot(i) = q_DotDot[i];
    }
    if(wrenches_ext.size() == 0){
        for(std::size_t i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            wrenches[i] = KDL::Wrench::Zero();
        }
    }
    else if(wrenches_ext.size() != chain_.getNrOfSegments()){
        ROS_ERROR_STREAM("Dimension mismatch: " << wrenches_ext.size() <<" wrenches are given as input, but kdl expects " << chain_.getNrOfSegments() << "wenches.");
        return KDL::SolverI::E_UNDEFINED;
    }
    else{
        for(std::size_t i = 0; i < chain_.getNrOfSegments(); ++i){
            wrenches[i] = wrenches_ext[i].toKDL();
        }
    }

    int e_code = solverID_.CartToJnt(qkdl,qkdl_Dot,qkdl_DotDot,wrenches,torques_kdl);

    convert(torques_kdl,torques);


    return e_code;
}

int Jaco2KinematicsDynamics::getFKPose(const std::vector<double> &q_in, tf::Pose &out, std::string link)
{
    KDL::JntArray q;
    KDL::Frame pose;
    KDL::ChainFkSolverPos_recursive fksolver(chain_);
    convert(q_in,q);
    int segId = getKDLSegmentIndexFK(link);
//    KDL::Segment s6 = chain_.getSegment(segId);
//    std::cout << s6.getName() << std::endl;
    if(segId > -2){
        int error_code = fksolver.JntToCart(q, pose, segId);
        double qx, qy, qz, qw;
        pose.M.GetQuaternion(qx,qy,qz,qw);
        out.setRotation(tf::Quaternion(qx, qy, qz, qw));
        out.setOrigin(tf::Vector3(pose.p(0), pose.p(1), pose.p(2)));
//        tf::PoseKDLToTF(pose,out);
        return error_code;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << "is not part of KDL chain.");
        return KDL::SolverI::E_UNDEFINED;
    }
}


int Jaco2KinematicsDynamics::getKDLSegmentIndexFK(const std::string &name) const
{
    for(int i=0; i < chain_.getNrOfSegments(); ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        if(seg.getName() == name)
        {
            if(i == chain_.getNrOfSegments() -1)
            {
                return -1;
            }
            else
            {
                return i+1;
            }
        }
    }
    return -1;
//    int i=0;
//    while (i < (int)chain_.getNrOfSegments()) {
//        if (chain_.getSegment(i).getName() == name) {
//            return i+1;
//        }
//        i++;
//    }
//    return -1;
}

void Jaco2KinematicsDynamics::convert(const KDL::JntArray &in, std::vector<double> &out)
{
    out.resize(in.rows());
    for(std::size_t i = 0; i < out.size(); ++i)
    {
        out[i] = in(i);
    }
}

void Jaco2KinematicsDynamics::convert(const std::vector<double> &in, KDL::JntArray &out)
{
    out.resize(in.size());
    for(std::size_t i = 0; i < out.rows(); ++i)
    {
        out(i) = in[i];
    }
}

