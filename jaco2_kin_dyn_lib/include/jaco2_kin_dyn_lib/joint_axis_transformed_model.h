#ifndef JOINT_AXIS_TRANSFORMED_MODEL_H
#define JOINT_AXIS_TRANSFORMED_MODEL_H
#include <jaco2_kin_dyn_lib/jaco2_modified_dynamic_model.h>
namespace Jaco2KinDynLib {

class JointAxisTransformedModel : public Jaco2ModifiedDynamicModel

{
public:
    JointAxisTransformedModel();
    JointAxisTransformedModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

protected:
    virtual void rnea(const KDL::JntArray &q,
                      const KDL::JntArray &q_dot,
                      const KDL::JntArray &q_dotdot,
                      const KDL::Wrenches& f_ext,
                      KDL::JntArray &torques) override;

    virtual void rnea(KDLJointStateData& data,
                      const KDL::Wrenches& f_ext,
                      std::vector<KDL::Wrench>& wrenches,
                      std::vector<KDL::Frame>& X) override;

};
}
#endif // JOINT_AXIS_TRANSFORMED_MODEL_H
