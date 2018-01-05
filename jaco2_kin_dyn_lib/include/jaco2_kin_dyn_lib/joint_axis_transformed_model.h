#ifndef JOINT_AXIS_TRANSFORMED_MODEL_H
#define JOINT_AXIS_TRANSFORMED_MODEL_H
#include <jaco2_kin_dyn_lib/jaco2_modified_dynamic_model.h>
namespace Jaco2KinDynLib {

class JointAxisTransformedModel : public Jaco2ModifiedDynamicModel

{
public:
    JointAxisTransformedModel();
    JointAxisTransformedModel(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);

private:
    void rnea(KDLJointStateData& data, const KDL::Wrenches& f_ext,
              std::vector<KDL::Wrench>& wrenches, std::vector<KDL::Twist>& S);
};
}
#endif // JOINT_AXIS_TRANSFORMED_MODEL_H
