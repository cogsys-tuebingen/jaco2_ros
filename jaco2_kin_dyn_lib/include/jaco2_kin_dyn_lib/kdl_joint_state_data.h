#ifndef KDLJOINTSTATEDATA_H
#define KDLJOINTSTATEDATA_H
#include <kdl/jntarray.hpp>
#include <jaco2_data/joint_state_data.h>
namespace Jaco2KinDynLib {
class KDLJointStateData
{
public:
    enum class DataType{
        JOINT_POS = 1,
        JOINT_VEL = 2,
        JOINT_ACC = 3,
        JOINT_TORQUE = 4
    };

    KDLJointStateData();
    KDLJointStateData(std::size_t n);
    KDLJointStateData(const jaco2_data::JointStateData& data, std::size_t ignore_end = 0);
    void resize(std::size_t n);

    void fromJaco2Data(const jaco2_data::JointStateData& data, std::size_t ignore_end = 0);
    jaco2_data::JointStateData toJointStateData();

    void setToZero();

public:
    int label;
    std::string frame_id;
    KDL::Vector gravity;

    jaco2_data::TimeStamp stamp;
    std::vector<std::string> names;
    KDL::JntArray position;
    KDL::JntArray velocity;
    KDL::JntArray acceleration;
    KDL::JntArray torque;
};
}
#endif // KDLJOINTSTATEDATA_H
