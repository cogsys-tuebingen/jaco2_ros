#include <ros/ros.h>
#include <gtest/gtest.h>
#include <jaco2_calibration_utils/dynamic_residual.h>

TEST(DynamicResidual,ALL)
{
    DynamicResidual model("robot_description",
                          "jaco_link_base",
                          "jaco_link_hand",
                          ResidualType::ALL);
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
