#include <jaco2_kin_dyn_lib/jaco2_modified_dynamic_model.h>
#include <jaco2_kin_dyn_lib/kdl_conversion.h>

using namespace Jaco2KinDynLib;
Jaco2ModifiedDynamicModel::Jaco2ModifiedDynamicModel() :
    has_transform_(false)
{
}

Jaco2ModifiedDynamicModel::Jaco2ModifiedDynamicModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip) :
    Jaco2DynamicModel(robot_model, chain_root, chain_tip),
    has_transform_(false)
{
}


int Jaco2ModifiedDynamicModel::getTorques(const std::vector<double> &q, const std::vector<double> &q_Dot, const std::vector<double> &q_DotDot,
                                          std::vector<double> &torques, const std::vector<KDL::Wrench> &wrenches_ext)
{
    std::size_t njoints = chain_.getNrOfJoints();
    if(q.size() != q_Dot.size() || q.size() != q_DotDot.size() || q.size() < njoints){
        std::stringstream e_des;
        e_des << "Dimension mismatch of input: Dimenion of q is "
              << q.size()        << ". While q_Dot has dimension "
              << q_Dot.size()    << " and q_DotDot "
              << q_DotDot.size() <<". The KDL chain contains "
              <<  chain_.getNrOfJoints() << "  joints." ;
        std::string error = e_des.str();
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    KDL::JntArray qkdl(njoints);
    KDL::JntArray qkdl_Dot(njoints);
    KDL::JntArray qkdl_DotDot(njoints);
    KDL::JntArray torques_kdl(njoints);
    KDL::Wrenches wrenches(njoints);
    for(std::size_t i = 0; i < njoints; ++i) {
        qkdl(i) = q[i];
        qkdl_Dot(i) = q_Dot[i];
        qkdl_DotDot(i) = q_DotDot[i];
    }

    if(wrenches_ext.size() == 0){
        for(std::size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
            wrenches[i] = KDL::Wrench::Zero();
        }
    }
    else if(wrenches_ext.size() != chain_.getNrOfSegments()) {
        std::stringstream e_des;
        e_des << "Dimension mismatch: " << wrenches_ext.size()
              <<" wrenches are given as input, but kdl expects " << chain_.getNrOfSegments() << "wenches.";
        std::string error = e_des.str();
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    rnea(qkdl,qkdl_Dot,qkdl_DotDot,wrenches_ext,torques_kdl);

    Jaco2KinDynLib::convert(torques_kdl,torques);
    return KDL::SolverI::E_NOERROR;
}

int Jaco2ModifiedDynamicModel::getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                                          std::vector<double>& torques, const std::vector<Wrench>& wrenches_ext)
{
    std::size_t njoints = chain_.getNrOfJoints();
    if(q.size() != q_Dot.size() || q.size() != q_DotDot.size() || q.size() < njoints){
        std::stringstream e_des;
        e_des << "Dimension mismatch of input: Dimenion of q is "
              << q.size()        << ". While q_Dot has dimension "
              << q_Dot.size()    << " and q_DotDot "
              << q_DotDot.size() <<". The KDL chain contains "
              <<  chain_.getNrOfJoints() << "  joints." ;
        std::string error = e_des.str();
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    KDL::JntArray qkdl(njoints);
    KDL::JntArray qkdl_Dot(njoints);
    KDL::JntArray qkdl_DotDot(njoints);
    KDL::JntArray torques_kdl(njoints);
    KDL::Wrenches wrenches;
    wrenches.resize(chain_.getNrOfSegments());
    for(std::size_t i = 0; i < njoints; ++i) {
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
        std::stringstream e_des;
        e_des << "Dimension mismatch: " << wrenches_ext.size()
              <<" wrenches are given as input, but kdl expects " << chain_.getNrOfSegments() << "wenches.";
        std::string error = e_des.str();
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    else{
        for(std::size_t i = 0; i < chain_.getNrOfSegments(); ++i){
            wrenches[i] = wrenches_ext[i].toKDL();
        }
    }
    rnea(qkdl,qkdl_Dot,qkdl_DotDot,wrenches,torques_kdl);

    Jaco2KinDynLib::convert(torques_kdl,torques);
    return KDL::SolverI::E_NOERROR;

}

void Jaco2ModifiedDynamicModel::setSensorTransforms(const std::vector<KDL::Frame> &transformation)
{
    std::size_t nj = chain_.getNrOfJoints();
    if(transformation.size() != nj){
        std::stringstream estream;
        estream << "Dimension mismatch: Chain contains " << nj << " joints. Thus, " << nj << " transformations are expected. Provide are "
                << transformation.size();
        throw std::logic_error(estream.str());
    }

    sensor_transforms_ = transformation;
    has_transform_ = true;
}

void Jaco2ModifiedDynamicModel::rnea(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Wrenches &f_ext, KDL::JntArray &torques)
{
    unsigned int j=0;
    std::size_t nj = chain_.getNrOfJoints();
    std::size_t ns = chain_.getNrOfSegments();

    if(!has_transform_) {
        sensor_transforms_.resize(nj);
        for(KDL::Frame& trans : sensor_transforms_) {
            trans.Identity();
        }
        has_transform_ = true;
    }

    KDL::Twist ag = -KDL::Twist(gravity_, KDL::Vector::Zero());  // as in orocos kdl
    std::vector<KDL::Wrench> f(ns);
    std::vector<KDL::Twist> a(ns);
    std::vector<KDL::Twist> S(ns);
    std::vector<KDL::Twist> v(ns);
    std::vector<KDL::Frame> X(ns);

    //Sweep from root to leaf
    for(std::size_t i=0; i < ns; ++i) {
        double q_, qdot_, qdotdot_;
        if(chain_.getSegment(i).getJoint().getType()!= KDL::Joint::None) {
            q_ = q(j);
            qdot_ = q_dot(j);
            qdotdot_ = q_dotdot(j);
            ++j;
        } else
            q_ = qdot_ = qdotdot_ = 0.0;

        //Calculate segment properties: X,S,vj,cj
        X[i] = chain_.getSegment(i).pose(q_);//Remark this is the inverse of the
        //frame for transformations from
        //the parent to the current coord frame
        //Transform velocity and unit velocity to segment frame
        KDL::Twist vj = X[i].M.Inverse(chain_.getSegment(i).twist(q_,qdot_));
        S[i] = X[i].M.Inverse(chain_.getSegment(i).twist(q_,1.0));
        //We can take cj=0, see remark section 3.5, page 55 since the unit velocity vector S of our joints is always time constant
        //calculate velocity and acceleration of the segment (in segment coordinates)
        if(i==0){
            v[i] = vj;
            a[i] = X[i].Inverse(ag) + S[i]*qdotdot_ + v[i]*vj;
        }else{
            v[i] = X[i].Inverse(v[i-1]) + vj;
            a[i] = X[i].Inverse(a[i-1]) + S[i]*qdotdot_ + v[i]*vj;
        }
        //Calculate the force for the joint
        //Collect RigidBodyInertia and external forces
        KDL::RigidBodyInertia Ii=chain_.getSegment(i).getInertia();
        f[i] = Ii*a[i] + v[i]*(Ii*v[i]) - f_ext[i];
    }
    //Sweep from leaf to root
    j = nj - 1;
    for(int i = ns-1; i >= 0; --i){
        if(chain_.getSegment(i).getJoint().getType() != KDL::Joint::None) {
            torques(j) = dot(S[i], sensor_transforms_[j] * f[i]);
            --j;
        }
        if(i!=0)
            f[i-1] = f[i-1] + X[i]*f[i];
    }
}
