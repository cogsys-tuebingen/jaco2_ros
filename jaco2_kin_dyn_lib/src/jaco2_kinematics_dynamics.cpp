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
//    solverFK_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

    //initialize TRAC_IK solver: inverse kinematics
//    solverIK_.reset(new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_));
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
                                                                         const std::vector<double> &q_DotDot)
{

    int tipId = getKDLSegmentIndexFK(tip);
    int rootId = getKDLSegmentIndexFK(root);
    // determine Matrix dimensions
    int nLinks = tipId - rootId + 1;

    if(nLinks > q.size() && nLinks > q_Dot.size() && nLinks > q_DotDot.size())
    {
        ROS_ERROR_STREAM("getRigidBodyRegressionMatrix: Dimension Mismatch: chain length and joint space vectors should have equal length.");
    }

    Eigen::MatrixXd result(nLinks,q.size()*10); // 10 is the total number of rigid body dynamic parameters 1 (mass) + 3 (center of mass) + 6 (inertia matrix)

    // for each link n between root and tip calculate regression matrices A_n see Handbook of Robotics EQ (14.41) page 331.
    // similar to newton - euler algorighm cf. KDL

    // calculate A_n in body coordinates;
    // Sweep from root to leaf
    int j = 0;
    std::vector<KDL::Frame> X(q.size());
    std::vector<KDL::Twist> S(q.size());
    std::vector<KDL::Twist> v;
    std::vector<KDL::Twist> a;
    std::vector<Wrench> f;
    std::vector<Eigen::Matrix<double, 6, 10> > An(q.size());
    KDL::Twist ag = -KDL::Twist(gravity_,KDL::Vector::Zero());

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
//            v[i]=vj;
//            a[i]=X[i].Inverse(ag)+S[i]*qdotdot_+v[i]*vj;
        }else{
            vparent = X[i].Inverse(v[i-1]);
            aparent = X[i].Inverse(a[i-1]);
//            v[i]=X[i].Inverse(v[i-1])+vj;
//            a[i]=X[i].Inverse(a[i-1])+S[i]*qdotdot_+v[i]*vj;
        }
        v[i] = vparent + vj;
        a[i] = aparent + S[i]*qdotdot_ + v[i]*vj;
        // Calculate the regression Matrix An for each Link
        // See Handbook of Robotics page 331
        KDL::Twist d = aparent - X[i].Inverse(ag); // see text page 331
        An[i].block<3,1>(0,0).setZero();
        An[i].block<3,3>(0,1) = -skewSymMat(d.vel);
        An[i].block<3,6>(0,4) = inertiaProductMat(a[i].rot) + skewSymMat(v[i].rot) * inertiaProductMat(v[i].rot);
        An[i].block<3,1>(4,0) = Eigen::Vector3d(d.vel(0), d.vel(1), d.vel(2));
        An[i].block<3,3>(4,1) = skewSymMat(a[i].rot) + skewSymMat(v[i].rot) * skewSymMat(v[i].rot);
        An[i].block<3,6>(4,4).setZero();
    }



    // Calculate Matrix K .transform matrices into each link n frame see Handbook of Robotics EQ (14.49) page 333.
    int col = 0;
//    for(unsigned int i = rootId; i <= tipID; ++i) {
//        result.block<
//    }
    // return K

    std::cerr << "not implemented, yet." << std::endl;

    return result;
}

Eigen::Matrix3d Jaco2KinematicsDynamicsModel::skewSymMat(const KDL::Vector &vec)
{
    Eigen::Matrix3d res;
    res << 0    , -vec(2)   , vec(1),
          vec(2), 0         , -vec(0),
         -vec(1), vec(1)    , 0;
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
