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
    if(q.size() != q_Dot.size() || q.size() != q_DotDot.size() || q.size() != chain_.getNrOfJoints()){
        ROS_ERROR_STREAM("Dimension mismatch of input: Dimenion of q is " << q.size() << ". While q_Dot has dimension " << q_Dot.size() << " and q_DotDot "
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
}

void Jaco2KinematicsDynamicsModel::getGravity(double &gx, double &gy, double &gz)
{
    gx = gravity_(0);
    gy = gravity_(1);
    gz = gravity_(2);
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

int Jaco2KinematicsDynamicsModel::getAcceleration(const double gx, const double gy, const double gz,
                                                  const std::vector<double>& q,
                                                  const std::vector<double>& q_Dot,
                                                  const std::vector<double>& q_DotDot,
                                                  std::vector<std::string>& links,
                                                  std::vector<KDL::Twist > &spatial_acc )
{

    if(q.size() != q_Dot.size() || q.size() != q_DotDot.size() || q.size() != chain_.getNrOfJoints()){
        ROS_ERROR_STREAM("Dimension mismatch of input: Dimenion of q is " << q.size() << ". While q_Dot has dimension " << q_Dot.size() << " and q_DotDot "
                         << q_DotDot.size()<<". The KDL chain contains " <<  chain_.getNrOfJoints() << "  joints." << std::endl);
        return KDL::SolverI::E_UNDEFINED;
    }

    std::vector<KDL::Frame> X(q.size());
    std::vector<KDL::Frame> Xij(q.size());
    std::vector<KDL::Twist> S(q.size());
    std::vector<KDL::Twist> v(q.size());
    std::vector<KDL::Twist> a(q.size());

    KDL::Twist ag = -KDL::Twist(KDL::Vector(gx,gy,gz),KDL::Vector::Zero());
    int j = 0;
    double ns = chain_.getNrOfSegments() ;
    //    std::cout << "ns " << ns << " nj " << chain_.getNrOfJoints() << std::endl;
    for(int i = 0; i < chain_.getNrOfJoints(); ++i) {
        double q_,qdot_,qdotdot_;
        if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
            q_=q[j];
            qdot_=q_Dot[j];
            qdotdot_=q_DotDot[j];
            ++j;
        }else
            q_=qdot_=qdotdot_=0.0;

        // Calculate segment properties: X,S,vj,cj
        X[i]=chain_.getSegment(i).pose(q_);//Remark this is the inverse of the
        // frame for transformations from
        // the parent to the current coord frame

        // Transform velocity and unit velocity to segment frame
        KDL::Twist vj = X[i].M.Inverse(chain_.getSegment(i).twist(q_,qdot_));

        KDL::Twist vparent;
        KDL::Twist aparent;
        if(i==0){
            vparent = KDL::Twist(KDL::Vector::Zero(), KDL::Vector::Zero());
            aparent = X[i].Inverse(ag);
            Xij[i] = X[i];
        }else{
            vparent = X[i].Inverse(v[i-1]);
            aparent = X[i].Inverse(a[i-1]);
            Xij[i] = Xij[i-1] * X[i];
        }
        v[i] = vparent + vj;
        a[i] = aparent + S[i]*qdotdot_ + v[i]*vj;

    }
    spatial_acc.resize(chain_.getNrOfSegments());
    links.resize(chain_.getNrOfSegments());
    for(std::size_t i = 0; i < links.size(); ++i) {
        links[i] = chain_.getSegment(i).getName();
        spatial_acc[i] = a[i];
    }
    return KDL::SolverI::E_NOERROR;
}

int Jaco2KinematicsDynamicsModel::getAcceleration(const double gx, const double gy, const double gz,
                                                  const std::vector<double> &q, const std::vector<double> &q_Dot,
                                                  const std::vector<double> &q_DotDot, std::vector<std::string> &links, std::vector<Eigen::Matrix<double, 6, 1>  >&spatial_acc)
{

    std::vector<KDL::Twist> acc;
    int ec = getAcceleration(gx,gy,gz,q,q_Dot, q_DotDot, links, acc);

    spatial_acc.resize(links.size());
    for(std::size_t i = 0; i < links.size(); ++i) {
        spatial_acc[i](0) = acc[i].rot(0);
        spatial_acc[i](1) = acc[i].rot(1);
        spatial_acc[i](2) = acc[i].rot(2);
        spatial_acc[i](3) = acc[i].vel(0);
        spatial_acc[i](4) = acc[i].vel(1);
        spatial_acc[i](5) = acc[i].vel(2);
    }
    return ec;

}
int Jaco2KinematicsDynamicsModel::getChainDynParam(const double gx, const double gy, const double gz,
                                                   const std::vector<double> &q,
                                                   const std::vector<double> &q_Dot,
                                                   Eigen::MatrixXd &H, Eigen::VectorXd &C, Eigen::VectorXd &G)
{
    if(q.size() != q_Dot.size() || q.size() != chain_.getNrOfJoints()){
        ROS_ERROR_STREAM("Dimension mismatch of input: Dimenion of q is " << q.size() << ". While q_Dot has dimension " << q_Dot.size()
                         <<". The KDL chain contains " <<  chain_.getNrOfJoints() << "  joints." << std::endl);
        return KDL::SolverI::E_UNDEFINED;
    }

    KDL::ChainDynParam dynparam(chain_, KDL::Vector(gx, gy, gz));

    KDL::JntArray theta;
    KDL::JntArray omega;
    KDL::JntArray coriolis(q.size());
    KDL::JntArray gravity(q.size());
    KDL::JntSpaceInertiaMatrix inertia(q.size());
    convert(q,theta);
    convert(q_Dot, omega);

    int res = dynparam.JntToCoriolis(theta, omega, coriolis);
    if(res == KDL::SolverI::E_NOERROR){
        res = dynparam.JntToGravity(theta, gravity);
        if(res == KDL::SolverI::E_NOERROR){
            res = dynparam.JntToMass(theta, inertia);
        }
        else{
            return res;
        }
    }
    else{
        return res;
    }

    kdlJntArray2Eigen(coriolis, C);
    kdlJntArray2Eigen(gravity, G);

    kdlMatrix2Eigen(inertia, H);

}

void Jaco2KinematicsDynamicsModel::changeDynamicParams(const std::string &link, const double mass, const Eigen::Vector3d &com, const Eigen::Matrix3d& inertia)
{
    int segId = getKDLSegmentIndex(link);

    if(segId > -1)
    {
        KDL::Vector cog(com(0), com(1), com(2));
        KDL::RotationalInertia inertiaKDL(inertia(0,0), inertia(1,1), inertia(2,2),
                                          inertia(0,1), inertia(0,2), inertia(1,2));

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

void Jaco2KinematicsDynamicsModel::changeDynamicParams(const std::string &link, const double mass, const Eigen::Vector3d &com, const Eigen::Matrix3d& inertia, double gx, double gy, double gz)
{
    gravity_ =KDL::Vector(gx, gy, gz);
    int segId = getKDLSegmentIndex(link);
    if(segId > -1)
    {
        KDL::Vector cog(com(0), com(1), com(2));
        KDL::RotationalInertia inertiaKDL(inertia(0,0), inertia(1,1), inertia(2,2),
                                          inertia(0,1), inertia(0,2), inertia(1,2));

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



void Jaco2KinematicsDynamicsModel::changeKineticParams(const std::string &link, const Eigen::Vector3d &trans, const Eigen::Matrix3d& rotation)
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
    if(name == root_) {
        return -1;
    }
    else {
        return -2;
    }
}

void Jaco2KinematicsDynamicsModel::getRandomConfig(std::vector<double>& config)
{
    KDL::JntArray ub, lb;
    solverIK_->getKDLLimits(lb, ub);
    config.resize(chain_.getNrOfJoints());
    for(std::size_t i = 0; i < config.size(); ++i){
        config[i] = jointDist_[i](randEng_);
    }
}

void Jaco2KinematicsDynamicsModel::getRandomConfig(Eigen::VectorXd& config)
{
    config.resize(chain_.getNrOfJoints());
    std::vector<double> rand;
    getRandomConfig(rand);
    for(std::size_t i = 0; i < rand.size(); ++i){
        config(i) = rand[i];
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
Eigen::Vector3d Jaco2KinematicsDynamicsModel::getLinkCoM(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Vector vec = chain_.getSegment(segmentID).getInertia().getCOG();
        Eigen::Vector3d result(vec(0), vec(1), vec(2));
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Vector3d ();
    }
}

Eigen::Matrix3d Jaco2KinematicsDynamicsModel::getLinkInertia(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::RotationalInertia mat = chain_.getSegment(segmentID).getInertia().getRotationalInertia();
        Eigen::Matrix3d result;
        result << mat.data[0], mat.data[1], mat.data[2],
                mat.data[3], mat.data[4], mat.data[5],
                mat.data[6], mat.data[7], mat.data[8];
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Matrix3d();
    }
}

Eigen::Matrix3d Jaco2KinematicsDynamicsModel::getLinkInertiaCoM(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::RotationalInertia mat = chain_.getSegment(segmentID).getInertia().getRotationalInertia();
        double mass = chain_.getSegment(segmentID).getInertia().getMass();
        KDL::Vector com = chain_.getSegment(segmentID).getInertia().getCOG();
        Eigen::Matrix3d result;

        double xx = mat.data[0] - mass *(com(1)*com(1) + com(2)*com(2));
        double yy = mat.data[4] - mass *(com(0)*com(0) + com(2)*com(2));
        double zz = mat.data[8] - mass *(com(0)*com(0) + com(1)*com(1));
        double xy = mat.data[1] + mass * com(0)*com(1);
        double xz = mat.data[2] + mass * com(0)*com(2);
        double yz = mat.data[5] + mass * com(1)*com(2);

        result << xx, xy, xz,
                xy, yy, yz,
                xz, yz, zz;
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Matrix3d();
    }
}

KDL::RotationalInertia Jaco2KinematicsDynamicsModel::getLinkInertiaCoM(const int id) const
{
    KDL::RotationalInertia mat = chain_.getSegment(id).getInertia().getRotationalInertia();
    double mass = chain_.getSegment(id).getInertia().getMass();
    KDL::Vector com = chain_.getSegment(id).getInertia().getCOG();

    double xx = mat.data[0] - mass *(com(1)*com(1) + com(2)*com(2));
    double yy = mat.data[4] - mass *(com(0)*com(0) + com(2)*com(2));
    double zz = mat.data[8] - mass *(com(0)*com(0) + com(1)*com(1));
    double xy = mat.data[1] + mass * com(0)*com(1);
    double xz = mat.data[2] + mass * com(0)*com(2);
    double yz = mat.data[5] + mass * com(1)*com(2);

    return KDL::RotationalInertia(xx, yy, zz, xy, xz, yz);
}



Eigen::Vector3d Jaco2KinematicsDynamicsModel::getLinkFixedTranslation(const std::string &link) const
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

Eigen::Matrix3d Jaco2KinematicsDynamicsModel::getLinkFixedRotation(const std::string &link) const
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

double Jaco2KinematicsDynamicsModel::getURDFLinkMass(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        return chainFile_.getSegment(segmentID).getInertia().getMass();
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return 0;
    }
}
Eigen::Vector3d Jaco2KinematicsDynamicsModel::getURDFLinkCoM(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::Vector vec = chainFile_.getSegment(segmentID).getInertia().getCOG();
        Eigen::Vector3d result(vec(0), vec(1), vec(2));
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Vector3d ();
    }
}



Eigen::Matrix3d Jaco2KinematicsDynamicsModel::getURDFLinkInertiaCoM(const std::string &link) const
{
    int segmentID = getKDLSegmentIndex(link);
    if(segmentID > -1){
        KDL::RotationalInertia mat = chainFile_.getSegment(segmentID).getInertia().getRotationalInertia();
        double mass = chainFile_.getSegment(segmentID).getInertia().getMass();
        KDL::Vector com = chainFile_.getSegment(segmentID).getInertia().getCOG();
        Eigen::Matrix3d result;

        double xx = mat.data[0] - mass *(com(1)*com(1) + com(2)*com(2));
        double yy = mat.data[4] - mass *(com(0)*com(0) + com(2)*com(2));
        double zz = mat.data[8] - mass *(com(0)*com(0) + com(1)*com(1));
        double xy = mat.data[1] + mass * com(0)*com(1);
        double xz = mat.data[2] + mass * com(0)*com(2);
        double yz = mat.data[5] + mass * com(1)*com(2);

        result << xx, xy, xz,
                xy, yy, yz,
                xz, yz, zz;
        return result;
    }
    else{
        ROS_ERROR_STREAM("Link " << link << " not found! Wrong name?");
        return Eigen::Matrix3d();
    }
}

Eigen::MatrixXd Jaco2KinematicsDynamicsModel::getRigidBodyRegressionMatrix(const std::string &root,
                                                                           const std::string &tip,
                                                                           const std::vector<double> &q,
                                                                           const std::vector<double> &q_Dot,
                                                                           const std::vector<double> &q_DotDot,
                                                                           const double& gx,
                                                                           const double& gy,
                                                                           const double& gz,
                                                                           const bool project)
{

    int tipId = getKDLSegmentIndex(tip);
    int rootId = getKDLSegmentIndex(root);
    // determine Matrix dimensions
    int nLinks = tipId - rootId + 1;

    if(q.size() != q_Dot.size() && q.size() != q_DotDot.size() && q.size() != chain_.getNrOfJoints()){
        ROS_ERROR_STREAM("Dimension mismatch of input: Dimenion of q is" << q.size() << ". While q_Dot has dimension " << q_Dot.size() << " and q_DotDot "
                         << q_DotDot.size()<<". The KDL chain contains " <<  chain_.getNrOfJoints() << "  joints." << std::endl);
        return Eigen::MatrixXd(0,0);
    }

    Eigen::MatrixXd result;

    if(project) {
        result = Eigen::MatrixXd(nLinks,nLinks*10); // 10 is the total number of rigid body dynamic parameters 1 (mass) + 3 (center of mass) + 6 (inertia matrix)
        // for each link n between root and tip calculate regression matrices A_n see Handbook of Robotics EQ (14.41) page 331.
        // similar to newton - euler algorighm cf. KDL
    }
    else {
        result = Eigen::MatrixXd(nLinks * 6, nLinks * 10); // if we have 3D force sensing in each joint
    }

    // calculate A_n in body coordinates;
    // Sweep from root to leaf
    int j = 0;
    std::vector<KDL::Frame> X(q.size());
    std::vector<KDL::Frame> Xij(q.size());
    std::vector<KDL::Twist> S(q.size());
    std::vector<KDL::Twist> v(q.size());
    std::vector<KDL::Twist> a(q.size());
    std::vector<Eigen::Matrix<double, 6, 10> > An(q.size());
    KDL::Twist ag = -KDL::Twist(KDL::Vector(gx,gy,gz),KDL::Vector::Zero());

    for(unsigned int i = 0; i < q.size(); ++i) {
        double q_,qdot_,qdotdot_;
        if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
            q_=q[j];
            qdot_=q_Dot[j];
            qdotdot_=q_DotDot[j];
            ++j;
        }else
            q_=qdot_=qdotdot_=0.0;

        // Calculate segment properties: X,S,vj,cj
        X[i]=chain_.getSegment(i).pose(q_);//Remark this is the inverse of the
        // frame for transformations from
        // the parent to the current coord frame

        // Transform velocity and unit velocity to segment frame
        KDL::Twist vj = X[i].M.Inverse(chain_.getSegment(i).twist(q_,qdot_));
        S[i]=X[i].M.Inverse(chain_.getSegment(i).twist(q_,1.0));
        // We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
        // calculate velocity and acceleration of the segment (in segment coordinates)

        KDL::Twist vparent;
        KDL::Twist aparent;
        if(i==0){
            vparent = KDL::Twist(KDL::Vector::Zero(), KDL::Vector::Zero());
            aparent = X[i].Inverse(ag);
            Xij[i] = X[i];
        }else{
            vparent = X[i].Inverse(v[i-1]);
            aparent = X[i].Inverse(a[i-1]);
            Xij[i] = Xij[i-1] * X[i];
        }
        v[i] = vparent + vj;
        a[i] = aparent + S[i]*qdotdot_ + v[i]*vj;

        // Calculate the regression Matrix An for each Link
        // See Handbook of Robotics page 331

        KDL::Vector d = a[i].vel + v[i].rot * v[i].vel; // - Xij[i].Inverse(ag); // see text page 331
        An[i].block<3,1>(0,0).setZero();
        An[i].block<3,3>(0,1) = -1.0 * skewSymMat(d);
        An[i].block<3,6>(0,4) = inertiaProductMat(a[i].rot) + skewSymMat(v[i].rot) * inertiaProductMat(v[i].rot);
        An[i].block<3,1>(3,0) = Eigen::Vector3d(d(0), d(1), d(2));
        An[i].block<3,3>(3,1) = skewSymMat(a[i].rot) + skewSymMat(v[i].rot) * skewSymMat(v[i].rot);
        An[i].block<3,6>(3,4).setZero();
    }

    // Calculate Matrix K .transform matrices into each link n frame see Handbook of Robotics EQ (14.49) page 333.
    for(int col = 0; col <nLinks; ++col) {
        for(int row = 0; row < nLinks; ++row) {
            if(col <= row) { // == nLinks -1 - row <= nLinks -1 - col
                KDL::Vector rotAxis = X[tipId - row].Inverse().M*chain_.getSegment(tipId - row).getJoint().JointAxis();
                Eigen::Matrix<double, 1, 6> zi;
                double norm =rotAxis.Norm();
                zi << rotAxis(0)/norm, rotAxis(1)/norm, rotAxis(2)/norm, 0, 0, 0;
                KDL::Frame xi = Xij[tipId - row].Inverse() * Xij[tipId - col];
                Eigen::Matrix<double, 6, 6> Xi = kdlFrame2Spatial(xi.Inverse()).transpose();
                if(project) {
                    Eigen::Matrix<double, 1, 10> Kij = zi * Xi * An[tipId - col];
                    result.block<1,10>( nLinks -1 - row,(nLinks -1 - col) * 10) = Kij;
                }
                else {
                    Eigen::Matrix<double, 6, 10> Aij = Xi * An[tipId - col];
                    result.block<6,10>( (nLinks -1 - row) * 6, (nLinks -1 - col) * 10) = Aij;
                }
            }
            else {
                if(project) {
                    result.block<1,10>(nLinks -1 - row,(nLinks -1 - col) * 10).setZero();
                }
                else {
                    result.block<6,10>((nLinks -1 - row) * 6,(nLinks -1 - col) * 10).setZero();
                }
            }

        }
    }

    return result;
}

void Jaco2KinematicsDynamicsModel::modifiedRNE(const double gx, const double gy, const double gz,
                                               const std::vector<double> &q1, const std::vector<double> &q2,
                                               const std::vector<double> &q3, const std::vector<double> &q4,
                                               Eigen::VectorXd &res)
{
    std::size_t nj = chain_.getNrOfJoints();
    std::size_t ns = chain_.getNrOfSegments();


    if(q1.size() != q2.size()
            && q2.size() != q3.size()
            &&  q3.size() != q4.size()
            && q4.size() != chain_.getNrOfJoints()){
        ROS_ERROR_STREAM("Dimension mismatch of input: Dimenion of q1 is" << q1.size() << ". While q2 has dimension " << q2.size() << ", q3 "
                         << q3.size() << " and q4 " << q4.size() <<". The KDL chain contains "
                         <<  chain_.getNrOfJoints() << "  joints." << std::endl);
        return;
    }

    unsigned int j=0;
    //Sweep from root to leaf
    KDL::Vector ag(-gx, -gy, -gz);

    std::vector<KDL::Frame> X(ns);
    std::vector<KDL::Twist> S(ns);
    std::vector<KDL::Vector> omega(ns);
    std::vector<KDL::Vector> omega_a(ns);;
    std::vector<KDL::Vector> omegaDot(ns);
    std::vector<KDL::Vector> a(ns);
    std::vector<KDL::Vector> F(ns);
    std::vector<KDL::Vector> N(ns);
    std::vector<KDL::Vector> n(ns);
    std::vector<KDL::Vector> f(ns);
    res.resize(nj);

    for(unsigned int i = 0; i < nj; ++i){
        double q_,qdot_, qdot_a_,qdotdot_;
        if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
            q_=q1[j];
            qdot_=q2[j];
            qdot_a_ =q3[j];
            qdotdot_=q4[j];
            j++;
        }else
            q_=qdot_=qdot_a_=qdotdot_=0.0;

        //Calculate segment properties: X,S
        X[i]=chain_.getSegment(i).pose(q_);//Remark this is the inverse of the
        //frame for transformations from
        //the parent to the current coord frame
        S[i]=X[i].M.Inverse(chain_.getSegment(i).twist(q_,1.0));

        // calculate angular velocity

        if(i == 0){
            omega[i]    = S[i].rot * qdot_ ;
            omega_a[i]  = S[i].rot * qdot_a_;
            omegaDot[i] = S[i].rot * qdotdot_;
            a[i]=X[i].Inverse().M*ag;
        }
        else{
            KDL::Frame Xij = X[i].Inverse();
            KDL::Rotation Eij = Xij.M;
            KDL::Vector rij = X[i].p;
            omega[i]    = Eij * omega[i-1] + S[i].rot * qdot_;                                                    // (AVR)
            omega_a[i]  = Eij * omega_a[i-1] + S[i].rot * qdot_a_;                                                // (AVR*)
            omegaDot[i] = Eij * omegaDot[i-1] + S[i].rot * qdotdot_ + (Eij * omega_a[i-1]) * (S[i].rot *  qdot_); // (AAR*0)
            a[i]        = Eij * (a[i-1] + omegaDot[i-1] * rij + omega[i-1] * (omega_a[i-1] * rij) );              // (LAR'*0)

        }
        // Calculate the force for the joint
        // Collect RigidBodyInertia no external forces
        KDL::RigidBodyInertia I = chain_.getSegment(i).getInertia();
        KDL::Vector cij = I.getCOG();
        F[i] = I.getMass() * (a[i] + omegaDot[i] * cij + omega[i] *(omega_a[i] * cij)); // body forces
        KDL::RotationalInertia Icm = getLinkInertiaCoM(i); // body torques
        N[i] = Icm * omegaDot[i] + omega_a[i] * (Icm * omega[i]);
    }

    //    Sweep from leaf to root
    j = nj -1;
    f[ns -1] = F[ns-1];
    n[ns -1] = N[ns-1] + chain_.getSegment(ns-1).getInertia().getCOG() * F[ns-1];

    for(int i = ns - 1; i >= 0; --i){
        if(i != 0){
            KDL::Rotation Eij = X[i].M;
            KDL::Vector fi = Eij* f[i];
            KDL::RigidBodyInertia I = chain_.getSegment(i-1).getInertia();
            f[i-1]  = F[i-1] + fi;
            n[i-1]  = N[i-1] + I.getCOG() * F[i-1]  + Eij * n[i]  + X[i].p * fi;
        }
        //        }
        if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None){
            res(j--) = dot(S[i].rot, n[i]);
        }
    }
    //    return (error = E_NOERROR);

}

Eigen::Matrix3d Jaco2KinematicsDynamicsModel::skewSymMat(const KDL::Vector &vec)
{
    Eigen::Matrix3d res;
    res << 0    , -vec(2)   , vec(1),
            vec(2), 0         , -vec(0),
            -vec(1), vec(0)    , 0;
    return res;
}

Eigen::Matrix<double, 3, 6> Jaco2KinematicsDynamicsModel::inertiaProductMat(const KDL::Vector &vec)
{
    Eigen::Matrix<double, 3, 6> res;
    res << vec(0), vec(1), vec(2), 0     , 0     , 0,
            0    , vec(0), 0     , vec(1), vec(2), 0,
            0    , 0     , vec(0), 0     , vec(1), vec(2);
    return res;

}

Eigen::Matrix<double, 6, 6> Jaco2KinematicsDynamicsModel::kdlFrame2Spatial(const KDL::Frame &frame)
{
    Eigen::Matrix<double, 6, 6> result;
    Eigen::Matrix<double, 3, 3> rot = kdlMatrix2Eigen(frame.M);
    result.block<3,3>(0,0) = rot;
    result.block<3,3>(0,3).setZero();
    result.block<3,3>(3,0) = skewSymMat(frame.p) * rot;
    result.block<3,3>(3,3) = rot;
    return result;
}

Eigen::Matrix<double, 3, 3> Jaco2KinematicsDynamicsModel::kdlMatrix2Eigen(const KDL::Rotation &rot)
{
    Eigen::Matrix<double, 3, 3> result;
    result << rot.data[0], rot.data[1], rot.data[2],
            rot.data[3], rot.data[4], rot.data[5],
            rot.data[6], rot.data[7], rot.data[8];
    return result;
}

void Jaco2KinematicsDynamicsModel::kdlJntArray2Eigen(const KDL::JntArray &q, Eigen::VectorXd& res)
{
    res.setZero(q.rows());
    for(std::size_t i = 0; i < q.rows(); ++i){
        res(i) = q(i);
    }
}

void Jaco2KinematicsDynamicsModel::kdlMatrix2Eigen(const KDL::JntSpaceInertiaMatrix& mat, Eigen::MatrixXd & res)
{
    res.setZero(mat.rows(), mat.columns());
    for(std::size_t i = 0; i < mat.rows(); ++i){
        for(std::size_t j = 0; j < mat.columns(); ++j){
            res(i,j) = mat(i,j);
        }
    }
}
