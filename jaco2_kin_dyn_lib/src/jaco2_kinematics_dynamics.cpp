#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>
#include <random>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>





Jaco2KinematicsDynamicsModel::Jaco2KinematicsDynamicsModel():
    gravity_(0,0,-9.81)//, solverID_(chain_,gravity_)
{
}

Jaco2KinematicsDynamicsModel::Jaco2KinematicsDynamicsModel(const std::string &robot_model, const std::string& chain_root, const std::string& chain_tip):
    urdf_param_(robot_model), root_(chain_root), tip_(chain_tip), gravity_(0,0,-9.81)//, solverID_(chain_,gravity_)
{

    ros::NodeHandle node_handle("~");

    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,urdf_param_);
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG_NAMED("Jaco2KinematicsDynamics","Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
    {
        ROS_FATAL_NAMED("Jaco2KinematicsDynamics","Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
    }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model_.initString(xml_string);
    initialize();

    std::cout << "Number of Joints: " << chain_.getNrOfJoints() << " | Number of Segments: " << chain_.getNrOfSegments() << std::endl;
}

void Jaco2KinematicsDynamicsModel::setTree(const std::string &robot_model)
{

    robot_model_.initString(robot_model);
    urdf_param_ = robot_model;
    initialize();
}

void Jaco2KinematicsDynamicsModel::initialize()
{
    if (!kdl_parser::treeFromUrdfModel(robot_model_, tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return;
    }
    if(tree_.getChain(root_,tip_,chain_)){
        chainFile_ = chain_;
        // inverse dynamics solver
        solverID_.reset(new KDL::ChainIdSolver_RNE(chain_,gravity_));

        // forward kinematic
        solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

        //initialize TRAC_IK solver: inverse kinematics
        solverIK_.reset(new TRAC_IK::TRAC_IK(root_, tip_, urdf_param_));

        solverIK_->getKDLLimits(lowerLimits_, upperLimits_);
        jointDist_.resize(chain_.getNrOfJoints());
        for(std::size_t i = 0; i < chain_.getNrOfJoints(); ++i){
            jointDist_[i] = std::uniform_real_distribution<double>(lowerLimits_(i),upperLimits_(i));
        }


    }
    else{
        ROS_ERROR("Chain extraction is not possible. Solver is not probably initalized");
    }
}

void::Jaco2KinematicsDynamicsModel::setRootAndTip(const std::string &chain_root, const std::string &chain_tip)
{
    root_ = chain_root;
    tip_ = chain_tip;
}

int Jaco2KinematicsDynamicsModel::getTorques(const std::vector<double> &q, const std::vector<double> &q_Dot, const std::vector<double> &q_DotDot,
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
    int e_code = -1;
    e_code = solverID_->CartToJnt(qkdl,qkdl_Dot,qkdl_DotDot,wrenches,torques_kdl);

    convert(torques_kdl,torques);


    return e_code;
}

void Jaco2KinematicsDynamicsModel::setGravity(double x, double y, double z)
{
    gravity_ = KDL::Vector(x, y, z);
    // inverse dynamics solver
    solverID_.reset(new KDL::ChainIdSolver_RNE(chain_,gravity_));

    // forward kinematic
    solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

    //initialize TRAC_IK solver: inverse kinematics
    solverIK_.reset(new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_));
}

int Jaco2KinematicsDynamicsModel::getFKPose(const std::vector<double> &q_in, tf::Pose &out, std::string link)
{
    KDL::JntArray q;
    KDL::Frame pose;

    convert(q_in,q);
    int segId = getKDLSegmentIndexFK(link);

    if(segId > -2){
        int error_code = solverFK_->JntToCart(q, pose, segId);
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

int Jaco2KinematicsDynamicsModel::getIKSolution(const tf::Pose& pose, std::vector<double>& result, const std::vector<double> &seed)
{
    KDL::JntArray q, solution;
    KDL::Frame frame;
    if(seed.size() == 0)
    {
        KDL::JntArray ub, lb;
        solverIK_->getKDLLimits(lb, ub);
        q.resize(chain_.getNrOfJoints());
        for(std::size_t i = 0; i < seed.size(); ++i){
            q(i) = jointDist_[i](randEng_);

        }
    }
    convert(seed,q);
    //convert tf pose to kdl frame
    PoseTFToKDL(pose,frame);
    int error_code = solverIK_->CartToJnt(q,frame,solution);
    convert(solution,result);
    return error_code;
}


void Jaco2KinematicsDynamicsModel::changeDynamicParams(const std::string &link, const double mass, const tf::Vector3 &com, const tf::Matrix3x3 inertia)
{
    int segId = getKDLSegmentIndex(link);

    if(segId > -1)
    {
        KDL::Vector cog(com.getX(), com.getY(), com.getZ());
        KDL::RotationalInertia inertiaKDL(inertia.getRow(0).getX(), inertia.getRow(1).getY(), inertia.getRow(2).getZ(),
                                          inertia.getRow(0).getY(), inertia.getRow(0).getZ(), inertia.getRow(1).getZ());

        KDL::Chain newChain;
        for(int i = 0; i < segId; ++i)
        {
            newChain.addSegment(chain_.getSegment(i));
        }
        KDL::Segment oldSeg = chain_.getSegment(segId);

        KDL::RigidBodyInertia newInertia(mass, cog, inertiaKDL);
        KDL::Segment newSeg(oldSeg.getName(),oldSeg.getJoint(), oldSeg.getFrameToTip(), newInertia);
        newChain.addSegment(newSeg);
        for(int i = segId +1; i < chain_.getNrOfSegments(); ++i)
        {
            newChain.addSegment(chain_.getSegment(i));
        }
        chain_ = newChain;

        // inverse dynamics solver
        solverID_.reset(new KDL::ChainIdSolver_RNE(chain_,gravity_));

    }
}

void Jaco2KinematicsDynamicsModel::changeDynamicParams(const std::string &link, const double mass, const tf::Vector3 &com, const tf::Matrix3x3 inertia, double gx, double gy, double gz)
{
    gravity_ =KDL::Vector(gx, gy, gz);
    int segId = getKDLSegmentIndex(link);
    if(segId > -1)
    {
        KDL::Vector cog(com.getX(), com.getY(), com.getZ());
        KDL::RotationalInertia inertiaKDL(inertia.getRow(0).getX(), inertia.getRow(1).getY(), inertia.getRow(2).getZ(),
                                          inertia.getRow(0).getY(), inertia.getRow(0).getZ(), inertia.getRow(1).getZ());

        KDL::Chain newChain;
        for(int i = 0; i < segId; ++i)
        {
            newChain.addSegment(chain_.getSegment(i));
        }
        KDL::Segment oldSeg = chain_.getSegment(segId);

        KDL::RigidBodyInertia newInertia(mass, cog, inertiaKDL);
        KDL::Segment newSeg(oldSeg.getName(),oldSeg.getJoint(), oldSeg.getFrameToTip(), newInertia);
        newChain.addSegment(newSeg);
        for(int i = segId +1; i < chain_.getNrOfSegments(); ++i)
        {
            newChain.addSegment(chain_.getSegment(i));
        }
        chain_ = newChain;

        // inverse dynamics solver
        solverID_.reset(new KDL::ChainIdSolver_RNE(chain_,gravity_));
    }
}



void Jaco2KinematicsDynamicsModel::changeKineticParams(const std::string &link, const tf::Vector3 &trans, const tf::Matrix3x3 rotation)
{
    {
        int segId = getKDLSegmentIndex(link);
        if(segId > -1)
        {
            KDL::Vector p(trans.getX(), trans.getY(), trans.getZ());
            //            KDL::Rotation rot(xx,yx,zx,xy,yy,zy,xz,yz,zz);
            KDL::Rotation rot(rotation.getColumn(0).getX(), rotation.getColumn(0).getY(), rotation.getColumn(0).getZ(),
                              rotation.getColumn(1).getX(), rotation.getColumn(1).getY(), rotation.getColumn(1).getZ(),
                              rotation.getColumn(2).getX(), rotation.getColumn(2).getY(), rotation.getColumn(2).getZ());

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
            // inverse dynamics solver
            solverID_.reset(new KDL::ChainIdSolver_RNE(chain_,gravity_));

            // forward kinematic
            solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

            //initialize TRAC_IK solver: inverse kinematics
            solverIK_.reset(new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_));
        }
    }
}

void Jaco2KinematicsDynamicsModel::useUrdfDynamicParams()
{
    chain_ = chainFile_;
    // inverse dynamics solver
    solverID_.reset(new KDL::ChainIdSolver_RNE(chain_,gravity_));

    // forward kinematic
    solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

    //initialize TRAC_IK solver: inverse kinematics
    solverIK_.reset(new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_));
}

int Jaco2KinematicsDynamicsModel::getKDLSegmentIndexFK(const std::string &name) const
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

int Jaco2KinematicsDynamicsModel::getKDLSegmentIndex(const std::string &name) const
{
    for(int i=0; i < chain_.getNrOfSegments(); ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        if(seg.getName() == name)
        {
            return i;
        }
    }
    return -1;
}

void Jaco2KinematicsDynamicsModel::getRandomConfig(std::vector<double>& config)
{
    KDL::JntArray ub, lb;
    solverIK_->getKDLLimits(lb, ub);
    config.resize(chain_.getNrOfJoints());
    for(std::size_t i = 0; i < config.size(); ++i){
        //        std::uniform_real_distribution<double> unif(lb(i),ub(i));
        //        std::default_random_engine re;
        //        config[i] = unif(re);
        config[i] = jointDist_[i](randEng_);
    }
}
double Jaco2KinematicsDynamicsModel::getLinkMass(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        return chain_.getSegment(segmentID).getInertia().getMass();
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return 0;
    }
}
tf::Vector3 Jaco2KinematicsDynamicsModel::getLinkCoM(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Vector vec = chain_.getSegment(segmentID).getInertia().getCOG();
        tf::Vector3 result(vec(0), vec(1), vec(2));
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return tf::Vector3();
    }
}

tf::Matrix3x3 Jaco2KinematicsDynamicsModel::getLinkInertia(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::RotationalInertia mat = chain_.getSegment(segmentID).getInertia().getRotationalInertia();
        tf::Matrix3x3 result;
        result.setValue(mat.data[0], mat.data[1], mat.data[2],
                mat.data[3], mat.data[4], mat.data[5],
                mat.data[6], mat.data[7], mat.data[8]);
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return tf::Matrix3x3();
    }
}

tf::Matrix3x3 Jaco2KinematicsDynamicsModel::getLinkInertiaCoM(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::RotationalInertia mat = chain_.getSegment(segmentID).getInertia().getRotationalInertia();
        double mass = chain_.getSegment(segmentID).getInertia().getMass();
        KDL::Vector com = chain_.getSegment(segmentID).getInertia().getCOG();
        tf::Matrix3x3 result;

        double xx = mat.data[0] - mass *(com(1)*com(1) + com(2)*com(2));
        double yy = mat.data[4] - mass *(com(0)*com(0) + com(2)*com(2));
        double zz = mat.data[8] - mass *(com(0)*com(0) + com(1)*com(1));
        double xy = mat.data[1] + mass * com(0)*com(1);
        double xz = mat.data[2] + mass * com(0)*com(2);
        double yz = mat.data[5] + mass * com(1)*com(2);

        result.setValue(xx, xy, xz,
                        xy, yy, yz,
                        xz, yz, zz);
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return tf::Matrix3x3();
    }
}

tf::Vector3 Jaco2KinematicsDynamicsModel::getLinkFixedTranslation(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Vector vec = chain_.getSegment(segmentID).getFrameToTip().p;
        tf::Vector3 result(vec(0), vec(1), vec(2));
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return tf::Vector3();
    }
}

tf::Matrix3x3 Jaco2KinematicsDynamicsModel::getLinkFixedRotation(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Rotation mat = chain_.getSegment(segmentID).getFrameToTip().M;
        tf::Matrix3x3 result;
        result.setValue(mat.data[0], mat.data[3], mat.data[6],
                mat.data[1], mat.data[4], mat.data[7],
                mat.data[2], mat.data[5], mat.data[8]);
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return tf::Matrix3x3();
    }
}

std::vector<std::string> Jaco2KinematicsDynamicsModel::getLinkNames() const
{
    std::vector<std::string> result;
    result.resize(chain_.getNrOfSegments());
    for(unsigned int i = 0; i < chain_.getNrOfSegments(); ++i){
        std::string name = chain_.getSegment(i).getName();
        result[i] = name;

    }
    return result;
}


void Jaco2KinematicsDynamicsModel::convert(const KDL::JntArray &in, std::vector<double> &out)
{
    out.resize(in.rows());
    for(std::size_t i = 0; i < out.size(); ++i)
    {
        out[i] = in(i);
    }
}

void Jaco2KinematicsDynamicsModel::convert(const std::vector<double> &in, KDL::JntArray &out)
{
    out.resize(in.size());
    for(std::size_t i = 0; i < out.rows(); ++i)
    {
        out(i) = in[i];
    }
}

void Jaco2KinematicsDynamicsModel::PoseTFToKDL(const tf::Pose& t, KDL::Frame& k)
{
    for (unsigned int i = 0; i < 3; ++i){
        k.p[i] = t.getOrigin()[i];
    }
    for (unsigned int i = 0; i < 9; ++i){
        k.M.data[i] = t.getBasis()[i/3][i%3];
    }
}
