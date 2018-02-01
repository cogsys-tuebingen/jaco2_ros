#include <ros/ros.h>
#include <gtest/gtest.h>
#include <jaco2_calibration_utils/dynamic_residual.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_data/joint_state_data.h>
TEST(DynamicResidual,ALL)
{
    DynamicResidual residual("robot_description",
                          "jaco_link_base",
                          "jaco_link_hand",
                          ResidualType::ALL);

    Jaco2KinDynLib::Jaco2DynamicModel model("robot_description",
                                            "jaco_link_base",
                                            "jaco_link_hand");

    std::vector<jaco2_data::JointStateData> data;
    for(int i = 0; i < 10 ; ++i){
        std::vector<double> q, qDot,qDotDot, torques;
        jaco2_data::JointStateDataStamped js;
        model.getRandomConfig(q);
        model.getRandomConfig(qDot);
        model.getRandomConfig(qDotDot);
        model.getTorques(q, qDot, qDotDot, torques);
        js.data.position = q;
        js.data.velocity = qDot;
        js.data.acceleration= qDotDot;
        js.data.torque = torques;
        js.header.frame_id = "";
        js.header.stamp.now();
        data.push_back(data);
    }

    residual.setData(d);
    Eigen::VectorXd param = residual.linSolve();
    Jaco2Calibration::to_Jaco2ManipulatorDynParams(param, model.getLinkNames(), paramout);
    auto it = paramout.begin();
    for(auto l :  model.getLinkNames()){
        auto com = model.getLinkCoM(l);
        for(int k = 0; k < 3; ++k){
            it->coM
        }
        auto m = model.getLinkMass(l);
        auto i = model.getLinkInertia(l);
    }


}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_dynamic_residual");
    ros::NodeHandle node("~");
    std::string robot_desc_string;
    //    node.getParam("/robot_description", robot_desc_string);//, std::string(""));
    std::string urdf_param("robot_description");
    return 0;
}
