#include <string>
#include <vector>
#include <math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_kin_dyn_lib/jaco2_modified_dynamic_model.h>
using namespace Jaco2KinDynLib;

Jaco2DynamicModel jaco2KDL;
Jaco2ModifiedDynamicModel modifiedModel;

TEST(Jaco2ModifiedDynamicsTests, IDtest)
{
    std::vector<double> q, qDot, qDotDot;
    jaco2KDL.setGravity(0,0,-9.81);
    modifiedModel.setGravity(0,0,-9.81);

    for(std::size_t i = 0; i < 1000000 ; ++ i){

        jaco2KDL.getRandomConfig(q);
        jaco2KDL.getRandomConfig(qDot);
        jaco2KDL.getRandomConfig(qDotDot);

        std::vector<double> tau1, tau2;
        jaco2KDL.getTorques(q, qDot, qDotDot,tau1);
        modifiedModel.getTorques(q, qDot, qDotDot,tau2);
        EXPECT_EQ(tau1.size(), tau2.size());

        for(int i = 0; i < tau1.size(); ++i) {
            EXPECT_NEAR(tau1[i], tau2[i], 1e-6);

        }
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_jaco2_dynamics");
    ros::NodeHandle node("~");
    std::string robot_desc_string;
    //    node.getParam("/robot_description", robot_desc_string);//, std::string(""));
    std::string urdf_param("robot_description");
    jaco2KDL = Jaco2DynamicModel(urdf_param,"jaco_link_base","jaco_link_hand");
    modifiedModel = Jaco2ModifiedDynamicModel(urdf_param,"jaco_link_base","jaco_link_hand");

    return RUN_ALL_TESTS();
}
