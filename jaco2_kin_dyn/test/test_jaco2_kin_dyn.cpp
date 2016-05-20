#include <string>
#include <vector>
#include <math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn/jaco2_kinematics_dynamics.h>


Jaco2KinematicsDynamics jaco2KDL;

TEST(Jaco2KinematicsDynamicsTest,converttest)
{
    KDL::JntArray q(6);
    std::vector<double> qvec;
    for(int i = 0; i< 6; ++i){
        q(i) = i;
    }
    Jaco2KinematicsDynamics::convert(q,qvec);
    for(int i =0; i<6; ++i){
        EXPECT_EQ(q(i),qvec[i]);
    }
    KDL::JntArray q2;
    Jaco2KinematicsDynamics::convert(qvec,q2);
    for(int i =0; i<6; ++i){
        EXPECT_EQ(q2(i),qvec[i]);
    }
}

TEST(Jaco2KinematicsDynamicsTest,chaintest)
{
    EXPECT_TRUE(jaco2KDL.getRootLink().find("jaco_link_base")!=std::string::npos);
    EXPECT_TRUE(jaco2KDL.getTipLink().find("jaco_link_hand")!=std::string::npos);
    EXPECT_EQ(jaco2KDL.getNrOfJoints(),6);
    EXPECT_EQ(jaco2KDL.getNrOfSegments(),6);
}

TEST(Jaco2KinematicsDynamicsTest,inverseDynamics)
{
    std::vector<double> q = {0, M_PI, M_PI, 0, 0, M_PI};
    std::vector<double> qDot;
    std::vector<double> torques;
    qDot.resize(6,0);
    torques.resize(6,0);
    int ec = jaco2KDL.getTorques(q,qDot,qDot,torques);
    EXPECT_TRUE(ec>=0);
    for(int i = 0; i <6; ++i)
    {
        std::cout << "torques(" << i <<") = " << torques[i] <<  std::endl;
    }
}

TEST(Jaco2KinematicsDynamicsTest, fk)
{
    std::vector<double> q = {0, M_PI, M_PI, 0, 0, M_PI};
    tf::Vector3 posH(-0.000, 0.063, 1.018);
    tf::Quaternion rotH(-0.707, 0.707, -0.000, 0.000);
    tf::Pose res;
    int ec = jaco2KDL.getFKPose(q,res,"jaco_link_hand");
    EXPECT_TRUE(ec>=0);
    EXPECT_NEAR(posH.getX(), res.getOrigin().getX(), 1e-3);
    EXPECT_NEAR(posH.getY(), res.getOrigin().getY(), 1e-3);
    EXPECT_NEAR(posH.getZ(), res.getOrigin().getZ(), 1e-3);
    EXPECT_NEAR(rotH.getX(), res.getRotation().getX(), 1e-3);
    EXPECT_NEAR(rotH.getY(), res.getRotation().getY(), 1e-3);
    EXPECT_NEAR(rotH.getZ(), res.getRotation().getZ(), 1e-3);
    EXPECT_NEAR(rotH.getW(), res.getRotation().getW(), 1e-3);

    tf::Vector3 pos5(-0.000, 0.023, 0.955);
    tf::Quaternion rot5(-0.612, 0.612, -0.354, -0.354);
    tf::Pose res5;
    ec = jaco2KDL.getFKPose(q,res5,"jaco_link_5");
    EXPECT_TRUE(ec>=0);
    EXPECT_NEAR(pos5.getX(), res5.getOrigin().getX(), 1e-3);
    EXPECT_NEAR(pos5.getY(), res5.getOrigin().getY(), 1e-3);
    EXPECT_NEAR(pos5.getZ(), res5.getOrigin().getZ(), 1e-3);
    EXPECT_NEAR(rot5.getX(), res5.getRotation().getX(), 1e-3);
    EXPECT_NEAR(rot5.getY(), res5.getRotation().getY(), 1e-3);
    EXPECT_NEAR(rot5.getZ(), res5.getRotation().getZ(), 1e-3);
    EXPECT_NEAR(rot5.getW(), res5.getRotation().getW(), 1e-3);


}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_jaco2_dynamics");
    ros::NodeHandle node("~");
    std::string robot_desc_string;
    node.getParam("/robot_description", robot_desc_string);//, std::string(""));
    jaco2KDL = Jaco2KinematicsDynamics(robot_desc_string,"jaco_link_base","jaco_link_hand");


    return RUN_ALL_TESTS();
}
