#ifndef JACO2_MODIFIED_DYNAMIC_MODEL_H
#define JACO2_MODIFIED_DYNAMIC_MODEL_H
#include "jaco2_dynamic_model.h"
#include "kdl_joint_state_data.h"

namespace Jaco2KinDynLib {

class Jaco2ModifiedDynamicModel : public Jaco2DynamicModel
{
public:
    Jaco2ModifiedDynamicModel();
    Jaco2ModifiedDynamicModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

    virtual int getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                   std::vector<double>& torques, const std::vector<jaco2_data::Wrench>& wrenches_ext = std::vector<jaco2_data::Wrench>()) override;

    virtual int getTorques(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot,
                   std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches_ext)  override;

    void getTorques(KDLJointStateData& data, std::vector<KDL::Wrench>& wrenches, std::vector<KDL::Twist>& S,
                    const std::vector<KDL::Wrench>& wrenches_ext = std::vector<KDL::Wrench>());


    void setSensorTransforms(const std::vector<KDL::Frame>& transformation);
    std::vector<KDL::Frame> getSensorTransforms() const {return sensor_transforms_;}

protected:
    void rnea(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Wrenches& f_ext,KDL::JntArray &torques);
    void rnea(KDLJointStateData& data, const KDL::Wrenches& f_ext,
              std::vector<KDL::Wrench>& wrenches, std::vector<KDL::Twist>& S);

    void checkInput(const std::vector<double>& q, const std::vector<double>& q_Dot, const std::vector<double>& q_DotDot);
    void checkInput(const KDLJointStateData& data);

protected:
    bool has_transform_;
    std::vector<KDL::Frame> sensor_transforms_;



};

}
#endif // JACO2_MODIFIED_DYNAMIC_MODEL_H
