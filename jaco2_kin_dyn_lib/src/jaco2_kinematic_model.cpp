/// Project
#include <jaco2_kin_dyn_lib/jaco2_kinematic_model.h>
#include <jaco2_kin_dyn_lib/kdl_conversion.h>
/// ROS
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>
/// Orocos KDL
#include <kdl/chainidsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
/// System
#include <random>
#include <boost/filesystem.hpp>
#include <jaco2_kin_dyn_lib/kdl_conversion.h>


using namespace Jaco2KinDynLib;

Jaco2KinematicModel::Jaco2KinematicModel():
    gravity_(0,0,-9.81)//, solverID_(chain_,gravity_)
{
}

Jaco2KinematicModel::Jaco2KinematicModel(const std::string &robot_model, const std::string& chain_root, const std::string& chain_tip):
    urdf_param_(robot_model), root_(chain_root), tip_(chain_tip), gravity_(0,0,-9.81)
{

    if(boost::filesystem::exists(robot_model)){
        ROS_DEBUG_NAMED("Jaco2KinematicModel","Load robot model from file.");
        if(!robot_model_.initFile(robot_model)){
            ROS_ERROR("Failed to pars urdf file");
        }
    }
    else{

        ros::NodeHandle node_handle("~");

        std::string xml_string;

        std::string urdf_xml,full_urdf_xml;
        node_handle.param("urdf_xml",urdf_xml,urdf_param_);
        node_handle.searchParam(urdf_xml,full_urdf_xml);

        ROS_DEBUG_NAMED("Jaco2KinematicModel","Reading xml file from parameter server");
        if (!node_handle.getParam(full_urdf_xml, xml_string))
        {
            ROS_FATAL_NAMED("Jaco2KinematicModel","Could not load the xml from parameter server: %s", urdf_xml.c_str());
            return;
        }

        node_handle.param(full_urdf_xml,xml_string,std::string());
        robot_model_.initString(xml_string);
    }
    initialize();

    ROS_DEBUG_STREAM_NAMED("Jaco2KinematicModel",
                           "Number of Joints: " << chain_.getNrOfJoints() <<
                           " | Number of Segments: " << chain_.getNrOfSegments());
}

Jaco2KinematicModel::~Jaco2KinematicModel()
{

}

bool Jaco2KinematicModel::setTreeParam(const std::string &robot_model)
{
    robot_model_.initString(robot_model);
    urdf_param_ = robot_model;
    return initialize();
}

bool Jaco2KinematicModel::setTreeFile(const std::string &robot_model)
{
    robot_model_.initFile(robot_model);
    urdf_param_ = robot_model_.getName();
    return initialize();
}



bool Jaco2KinematicModel::initialize()
{
    if (!kdl_parser::treeFromUrdfModel(robot_model_, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    if(tree_.getChain(root_,tip_,chain_)){
        chainFile_ = chain_;
        // forward kinematic
        solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

        // jacobian solver (calculates jacobian for given joint configuration)
        solverJac_.reset(new KDL::ChainJntToJacSolver(chain_));
        // inverse vel solver
        solverIKVel_.reset(new KDL::ChainIkSolverVel_pinv(chain_));

        //initialize TRAC_IK solver: inverse kinematics
        solverIK_.reset(new TRAC_IK::TRAC_IK(root_, tip_, urdf_param_));

        solverIK_->getKDLLimits(lowerLimits_, upperLimits_);
        jointDist_.resize(chain_.getNrOfJoints());
        for(std::size_t i = 0; i < chain_.getNrOfJoints(); ++i){
            jointDist_[i] = std::uniform_real_distribution<double>(lowerLimits_(i),upperLimits_(i));
        }

    }
    else{
        ROS_WARN_STREAM("Chain extraction is not possible. Solver is not probably initalized. robot_model: " << robot_model_.name_
                                         << "; Root link: "<< root_ << "; tip: " << tip_);
        return false;
    }
    return true;
}

void::Jaco2KinematicModel::setRootAndTip(const std::string &chain_root, const std::string &chain_tip)
{
    root_ = chain_root;
    tip_ = chain_tip;
}


int Jaco2KinematicModel::getFKPose(const std::vector<double> &q_in, KDL::Frame &out, const std::string link) const
{
    if(q_in.size() < chain_.getNrOfJoints()){
        //        std::string error = std::to_string(chain_.getNrOfJoints()) + " joint values expected got only " + std::to_string(q_in.size()) + "!";
        //        throw std::runtime_error(error);
        ROS_ERROR_STREAM(chain_.getNrOfJoints() << " joint values expected got only " << q_in.size() << "!");
        return KDL::SolverI::E_UNDEFINED;
    }

    KDL::JntArray q;
    Jaco2KinDynLib::convert(q_in, q, q_in.size() - chain_.getNrOfJoints());

    int segId = getKDLSegmentIndexFK(link);

    if(segId > -2){
        int error_code = solverFK_->JntToCart(q, out, segId);
        return error_code;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " is not part of KDL chain.");
        return KDL::SolverI::E_UNDEFINED;
    }
}

int Jaco2KinematicModel::getFKPose(const std::vector<double> &q_in, tf::Pose &out, const std::string link) const
{
    KDL::Frame pose;

    int error_code = getFKPose(q_in, pose, link);
    double qx, qy, qz, qw;
    pose.M.GetQuaternion(qx,qy,qz,qw);
    out.setRotation(tf::Quaternion(qx, qy, qz, qw));
    out.setOrigin(tf::Vector3(pose.p(0), pose.p(1), pose.p(2)));
    return error_code;

}

int Jaco2KinematicModel::getIKSolution(const tf::Pose& pose, std::vector<double>& result, const std::vector<double> &seed)
{
    KDL::JntArray q(chain_.getNrOfJoints());
    KDL::JntArray solution(chain_.getNrOfJoints());
    KDL::Frame frame;
    if(seed.size() == 0){
        KDL::JntArray ub, lb;
        solverIK_->getKDLLimits(lb, ub);
        q.resize(chain_.getNrOfJoints());
        for(std::size_t i = 0; i < chain_.getNrOfJoints(); ++i){
            q(i) = jointDist_[i](randEng_);

        }
    } else{
        Jaco2KinDynLib::convert(seed,q);
    }
    //convert tf pose to kdl frame
    Jaco2KinDynLib::poseTFToKDL(pose,frame);
    int error_code = solverIK_->CartToJnt(q,frame,solution);
    Jaco2KinDynLib::convert(solution,result);
    return error_code;
}

void Jaco2KinematicModel::changeKineticParams(const std::string &link, const Eigen::Vector3d &trans, const Eigen::Matrix3d& rotation)
{
    {
        int segId = getKDLSegmentIndex(link);
        if(segId > -1)
        {
            KDL::Vector p(trans(0), trans(1), trans(2));
            //            KDL::Rotation rot(xx,yx,zx,xy,yy,zy,xz,yz,zz);
            KDL::Rotation rot(rotation(0,0), rotation(1,0), rotation(2,0),
                              rotation(0,1), rotation(1,1), rotation(2,1),
                              rotation(0,2), rotation(1,2), rotation(2,2));

            KDL::Chain newChain;
            for(int i = 0; i < segId; ++i)
            {
                newChain.addSegment(chain_.getSegment(i));
            }
            KDL::Segment oldSeg = chain_.getSegment(segId);
            KDL::Frame matT(rot,p);
            KDL::Segment newSeg(oldSeg.getName(),oldSeg.getJoint(), matT, oldSeg.getInertia());
            newChain.addSegment(newSeg);
            for(int i = segId +1; i < chain_.getNrOfSegments(); ++i)
            {
                newChain.addSegment(chain_.getSegment(i));
            }
            chain_ = newChain;

            // forward kinematic
            solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

            //initialize TRAC_IK solver: inverse kinematics
            solverIK_.reset(new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_));
        }
    }
}

void Jaco2KinematicModel::useUrdfDynamicParams()
{
    chain_ = chainFile_;

    // forward kinematic
    solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

    //initialize TRAC_IK solver: inverse kinematics
    solverIK_.reset(new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_));
}

int Jaco2KinematicModel::getKDLSegmentIndexFK(const std::string &name) const
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
    return -2;
}

int Jaco2KinematicModel::getKDLSegmentIndex(const std::string &name) const
{
    for(int i=0; i < chain_.getNrOfSegments(); ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        if(seg.getName() == name)
        {
            return i;
        }
    }
    if(name == root_) {
        return -1;
    }
    else {
        return -2;
    }
}

void Jaco2KinematicModel::getRandomConfig(std::vector<double>& config)
{
    KDL::JntArray ub, lb;
    solverIK_->getKDLLimits(lb, ub);
    config.resize(chain_.getNrOfJoints());
    for(std::size_t i = 0; i < config.size(); ++i){
        config[i] = jointDist_[i](randEng_);
    }
}

void Jaco2KinematicModel::getRandomConfig(Eigen::VectorXd& config)
{
    config.resize(chain_.getNrOfJoints());
    std::vector<double> rand;
    getRandomConfig(rand);
    for(std::size_t i = 0; i < rand.size(); ++i){
        config(i) = rand[i];
    }
}


Eigen::Vector3d Jaco2KinematicModel::getLinkFixedTranslation(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Vector vec = chain_.getSegment(segmentID).getFrameToTip().p;
        Eigen::Vector3d result(vec(0), vec(1), vec(2));
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Vector3d();
    }
}

Eigen::Matrix3d Jaco2KinematicModel::getLinkFixedRotation(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Rotation mat = chain_.getSegment(segmentID).getFrameToTip().M;
        Eigen::Matrix3d result;
        result << mat.data[0], mat.data[3], mat.data[6],
                mat.data[1], mat.data[4], mat.data[7],
                mat.data[2], mat.data[5], mat.data[8];
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Matrix3d();
    }
}

std::vector<std::string> Jaco2KinematicModel::getLinkNames() const
{
    std::vector<std::string> result;
    result.resize(chain_.getNrOfSegments());
    for(unsigned int i = 0; i < chain_.getNrOfSegments(); ++i){
        std::string name = chain_.getSegment(i).getName();
        result[i] = name;

    }
    return result;
}

double  Jaco2KinematicModel::getUpperJointLimit(const std::size_t id) const
{
    if(upperLimits_.rows() > id){
        return upperLimits_(id);
    }
    else{
        return 0;
    }
}

double  Jaco2KinematicModel::getLowerJointLimit(const std::size_t id) const
{
    if(lowerLimits_.rows() > id){
        return lowerLimits_(id);
    }
    else{
        return 0;
    }
}

void Jaco2KinematicModel::getRotationAxis(const std::string &link, KDL::Vector& rot_axis) const
{
    int id = getKDLSegmentIndex(link);
    KDL::Frame X = chain_.getSegment(id).pose(0);
    rot_axis = X.Inverse().M * chain_.getSegment(id).getJoint().JointAxis();
}

KDL::Twist Jaco2KinematicModel::getJointAxisProjection(const std::string &link) const
{
    int i = getKDLSegmentIndex(link);
    KDL::Frame X = chain_.getSegment(i).pose(0);//Remark this is the inverse of the
    KDL::Twist S = X.M.Inverse(chain_.getSegment(i).twist(0,1.0));
    return S;
}

std::vector<KDL::Twist> Jaco2KinematicModel::getJointAxisProjections() const
{
    int ns = chain_.getNrOfSegments();
    std::vector<KDL::Twist> res(ns);
    auto it = res.begin();
    for(int i = 0;  i < ns; i++, it++){;
        KDL::Frame X = chain_.getSegment(i).pose(0);//Remark this is the inverse of the
        (*it) = X.M.Inverse(chain_.getSegment(i).twist(0,1.0));
    }
    return res;
}
void Jaco2KinematicModel::getRotationAxis(const std::string &link, Eigen::Vector3d &rot_axis) const
{
    KDL::Vector v;
    getRotationAxis(link, v);
    rot_axis(0) = v(0);
    rot_axis(1) = v(1);
    rot_axis(2) = v(2);
}

KDL::Jacobian Jaco2KinematicModel::getJacobian(const std::vector<double> &q, int seg_id) const
{
    std::size_t nj = chain_.getNrOfJoints();
    if(q.size() < nj ){
        throw std::logic_error("Dimension mismatch. More joint values expected");
    }
    KDL::Jacobian jac(nj);

    KDL::JntArray qvals;
    Jaco2KinDynLib::convert(q, qvals, q.size() - nj);

    solverJac_->JntToJac(qvals, jac, seg_id);
    return jac;
}

int Jaco2KinematicModel::getJointVelocities(const std::vector<double> &q, const KDL::Twist &v_in, std::vector<double>& v_out)
{
    std::size_t nj = chain_.getNrOfJoints();
    if(q.size() < nj ){
        throw std::logic_error("Dimension mismatch. More joint values expected");
    }
    KDL::JntArray qvals;
    Jaco2KinDynLib::convert(q, qvals, q.size() - nj);

    KDL::JntArray out;
    out.resize(nj);
    int ec = solverIKVel_->CartToJnt(qvals,v_in, out);
    Jaco2KinDynLib::convert(out, v_out);

    return ec;

}
