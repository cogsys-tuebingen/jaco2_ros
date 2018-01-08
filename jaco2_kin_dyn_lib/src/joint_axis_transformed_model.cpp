#include <jaco2_kin_dyn_lib/joint_axis_transformed_model.h>

using namespace Jaco2KinDynLib;
JointAxisTransformedModel::JointAxisTransformedModel()
{
}

JointAxisTransformedModel::JointAxisTransformedModel(const std::string &robot_model, const std::string &chain_root, const std::string &chain_tip) :
    Jaco2ModifiedDynamicModel(robot_model, chain_root, chain_tip)
{
}

void JointAxisTransformedModel::rnea(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Wrenches &f_ext, KDL::JntArray &torques)
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
            torques(j) = dot(S[i], f[i]);
            --j;
        }
        if(i!=0)
            f[i-1] = sensor_transforms_[i] * ( f[i-1] +  X[i]*f[i] );
    }
}

void JointAxisTransformedModel::rnea(KDLJointStateData& data, const KDL::Wrenches& f_ext,
                             std::vector<KDL::Wrench>& f, std::vector<KDL::Frame>& X)
{
    unsigned int j=0;
    std::size_t nj = chain_.getNrOfJoints();
    std::size_t ns = chain_.getNrOfSegments();
    data.torque.resize(nj);

    if(!has_transform_) {
        sensor_transforms_.resize(nj);
        for(KDL::Frame& trans : sensor_transforms_) {
            trans.Identity();
        }
        has_transform_ = true;
    }

    KDL::Twist ag = -KDL::Twist(gravity_, KDL::Vector::Zero());  // as in orocos kdl
    f.resize(ns);
    std::vector<KDL::Twist> a(ns);
    X.resize(ns);
    std::vector<KDL::Twist> v(ns);
    std::vector<KDL::Twist> S(ns);


    //Sweep from root to leaf
    for(std::size_t i=0; i < ns; ++i) {
        double q_, qdot_, qdotdot_;
        if(chain_.getSegment(i).getJoint().getType()!= KDL::Joint::None) {
            q_ = data.position(j);
            qdot_ = data.velocity(j);
            qdotdot_ = data.acceleration(j);
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
    std::vector<KDL::Wrench> fprime = f;
    //Sweep from leaf to root
    j = nj - 1;
    for(int i = ns-1; i >= 0; --i){
        if(chain_.getSegment(i).getJoint().getType() != KDL::Joint::None) {
            data.torque(j) = dot(S[i],  fprime[i]);
            --j;
        }
        if(i!=0)
            fprime[i-1] = sensor_transforms_[i] * ( fprime[i-1] + X[i]*fprime[i] );
    }
}
