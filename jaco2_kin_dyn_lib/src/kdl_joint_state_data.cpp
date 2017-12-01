#include <jaco2_kin_dyn_lib/kdl_joint_state_data.h>
#include <jaco2_kin_dyn_lib/kdl_conversion.h>
using namespace Jaco2KinDynLib;
KDLJointStateData::KDLJointStateData():
    gravity(0,0,-9.81)
{

}

KDLJointStateData::KDLJointStateData(std::size_t n) :
    gravity(0,0,-9.81),
    position(n),
    velocity(n),
    acceleration(n),
    torque(n)

{
}

KDLJointStateData::KDLJointStateData(const jaco2_data::JointStateData &data, std::size_t ignore_end) :
    label(data.label),
    frame_id(data.frame_id),
    gravity(data.gravity(0),data.gravity(1),data.gravity(2)),
    stamp(data.stamp)
{
    convert(data.position, position, ignore_end);
    convert(data.velocity, velocity, ignore_end);
    convert(data.acceleration, acceleration, ignore_end);
    convert(data.torque, torque, ignore_end);
    for(std::size_t i = 0; i < data.names.size() - ignore_end; ++i){
        names.push_back(data.names[i]);
    }

}

void KDLJointStateData::resize(std::size_t n)
{
    position.resize(n);
    velocity.resize(n);
    acceleration.resize(n);
    torque.resize(n);
    setToZero();
}

void KDLJointStateData::fromJaco2Data(const jaco2_data::JointStateData &data, std::size_t ignore_end)
{
    convert(data.position, position, ignore_end);
    convert(data.velocity, velocity, ignore_end);
    convert(data.acceleration, acceleration, ignore_end);
    convert(data.torque, torque, ignore_end);
    label = data.label;
    frame_id =data.frame_id;
    gravity = KDL::Vector(data.gravity(0),data.gravity(1),data.gravity(2));
    stamp = data.stamp;
    for(std::size_t i = 0; i < data.names.size() - ignore_end; ++i){
        names.push_back(data.names[i]);
    }
}

jaco2_data::JointStateData KDLJointStateData::toJointStateData()
{
    jaco2_data::JointStateData res;
    res.frame_id = frame_id;
    res.label = label;
    res.stamp = stamp;
    res.names = names;
    res.gravity = Eigen::Vector3d(gravity(0),gravity(1),gravity(2));
    convert(position, res.position);
    convert(velocity, res.velocity);
    convert(acceleration, res.acceleration);
    convert(torque, res.torque);
    return res;
}


void KDLJointStateData::setToZero()
{
    SetToZero(position);
    SetToZero(velocity);
    SetToZero(acceleration);
    SetToZero(torque);
}
