#include <string>
#include <vector>
#include <math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn/jaco2_kinematics_dynamics.h>


Jaco2KinematicsDynamicsModel jaco2KDL;

TEST(Jaco2KinematicsDynamicsModelTest,converttest)
{
    KDL::JntArray q(6);
    std::vector<double> qvec;
    for(int i = 0; i< 6; ++i){
        q(i) = i;
    }
    Jaco2KinematicsDynamicsModel::convert(q,qvec);
    for(int i =0; i<6; ++i){
        EXPECT_EQ(q(i),qvec[i]);
    }
    KDL::JntArray q2;
    Jaco2KinematicsDynamicsModel::convert(qvec,q2);
    for(int i =0; i<6; ++i){
        EXPECT_EQ(q2(i),qvec[i]);
    }
}

TEST(Jaco2KinematicsDynamicsModelTest,chaintest)
{
    EXPECT_TRUE(jaco2KDL.getRootLink().find("jaco_link_base")!=std::string::npos);
    EXPECT_TRUE(jaco2KDL.getTipLink().find("jaco_link_hand")!=std::string::npos);
    EXPECT_EQ(jaco2KDL.getNrOfJoints(),6);
    EXPECT_EQ(jaco2KDL.getNrOfSegments(),6);
}

TEST(Jaco2KinematicsDynamicsModelTest, DynParam)
{
    tf::Vector3 comLink1 = jaco2KDL.getLinkCoM("jaco_link_1");
    EXPECT_NEAR(comLink1.getX(), 0, 1e-4);
    EXPECT_NEAR(comLink1.getY(),  0.0086, 1e-4);
    EXPECT_NEAR(comLink1.getZ(), -0.1064, 1e-4);
    EXPECT_NEAR(jaco2KDL.getLinkMass("jaco_link_1"),0.7417, 1e-4);
    tf::Matrix3x3 inertia = jaco2KDL.getLinkInertia("jaco_link_1");
    EXPECT_NEAR(inertia.getColumn(0).getX(), 0.0094, 1e-4);
    EXPECT_NEAR(inertia.getColumn(0).getY(), 0.0000, 1e-4);
    EXPECT_NEAR(inertia.getColumn(0).getZ(), 0.0000, 1e-4);
    EXPECT_NEAR(inertia.getColumn(1).getY(), 0.0522, 1e-4);
    EXPECT_NEAR(inertia.getColumn(1).getZ(), 0.0006, 1e-4);
    EXPECT_NEAR(inertia.getColumn(2).getZ(), 0.0004, 1e-4);

    tf::Vector3 comLink3 = jaco2KDL.getLinkCoM("jaco_link_3");
    EXPECT_NEAR(comLink3.getX(),  0.1624, 1e-4);
    EXPECT_NEAR(comLink3.getY(),  0.0000, 1e-4);
    EXPECT_NEAR(comLink3.getZ(), -0.0135, 1e-4);
    EXPECT_NEAR(jaco2KDL.getLinkMass("jaco_link_3"), 0.7847, 1e-4);
    inertia = jaco2KDL.getLinkInertia("jaco_link_3");
    EXPECT_NEAR(inertia.getColumn(0).getX(),  0.0004, 1e-4);
    EXPECT_NEAR(inertia.getColumn(0).getY(),  0.0000, 1e-4);
    EXPECT_NEAR(inertia.getColumn(0).getZ(),  0.0015, 1e-4);
    EXPECT_NEAR(inertia.getColumn(1).getY(),  0.0243, 1e-4);
    EXPECT_NEAR(inertia.getColumn(1).getZ(),  0.0000, 1e-4);
    EXPECT_NEAR(inertia.getColumn(2).getZ(),  0.0243, 1e-4);

}
TEST(Jaco2KinematicsDynamicsModelTEST, KinParam)
{
    tf::Vector3 trans = jaco2KDL.getLinkFixedTranslation("jaco_link_1");
    tf::Matrix3x3 rot = jaco2KDL.getLinkFixedRotation("jaco_link_1");
    EXPECT_NEAR(trans.getX(), 0, 1e-4);
    EXPECT_NEAR(trans.getY(), 0, 1e-4);
    EXPECT_NEAR(trans.getZ(), 0.1535, 1e-4);
    EXPECT_NEAR(rot.getRow(0).getX(),  1, 1e-4);
    EXPECT_NEAR(rot.getRow(0).getY(),  0, 1e-4);
    EXPECT_NEAR(rot.getRow(0).getZ(),  0, 1e-4);
    EXPECT_NEAR(rot.getRow(1).getX(),  0, 1e-4);
    EXPECT_NEAR(rot.getRow(1).getY(), -1, 1e-4);
    EXPECT_NEAR(rot.getRow(1).getZ(),  0, 1e-4);
    EXPECT_NEAR(rot.getRow(2).getX(),  0, 1e-4);
    EXPECT_NEAR(rot.getRow(2).getY(),  0, 1e-4);
    EXPECT_NEAR(rot.getRow(2).getZ(), -1, 1e-4);
}

TEST(Jaco2KinematicsDynamicsModelTest,inverseDynamics)
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
        EXPECT_NEAR(torques[i], 0, 1e-4);
        std::cout << "torques(" << i <<") = " << torques[i] <<  std::endl;
    }
     std::vector<double> q2 = {4.776098185246001, 2.9779051856135985, 1.0370221453036228, -2.089493953881068, 1.3567380962770526, 1.3811624792663872};
     std::vector<double> qDot2;
     qDot2.resize(6,0.1);
     ec = jaco2KDL.getTorques(q2,qDot,qDot,torques);
     for(int i = 0; i <6; ++i)
     {
         if(i>0){
             EXPECT_TRUE(fabs(torques[i]) > 1e-4);
         }
         std::cout << "torques(" << i <<") = " << torques[i] <<  std::endl;
     }
}

TEST(Jaco2KinematicsDynamicsModelTest, fk)
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

TEST(Jaco2KinematicsDynamicsModelTest, changeDynParam)
{
    std::string linkName("jaco_link_hand");
    tf::Vector3 com(1.1, 0.1, 0.1);
    tf::Matrix3x3 mat(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);
    jaco2KDL.changeDynamicParams(linkName,jaco2KDL.getLinkMass(linkName), com, mat);
    double mass = jaco2KDL.getLinkMass(linkName);
    tf::Vector3 v = jaco2KDL.getLinkCoM(linkName);
    tf::Matrix3x3 mat2 = jaco2KDL.getLinkInertia(linkName);
    EXPECT_NEAR(v.getX(),com.getX(), 1e-6);
    EXPECT_NEAR(v.getY(),com.getY(), 1e-6);
    EXPECT_NEAR(v.getZ(),com.getZ(), 1e-6);
    EXPECT_EQ(mat2.getRow(0).getX(), mat.getRow(0).getX() + mass*(com.getY()*com.getY() + com.getZ() * com.getZ()));
    EXPECT_EQ(mat2.getRow(0).getY(), mat.getRow(0).getY() - mass*com.getX()*com.getY());
    EXPECT_EQ(mat2.getRow(0).getZ(), mat.getRow(0).getZ() - mass*com.getX()*com.getZ());
    EXPECT_EQ(mat2.getRow(1).getY(), mat.getRow(1).getY() + mass*(com.getX()*com.getX() + com.getZ() * com.getZ()));
    EXPECT_EQ(mat2.getRow(1).getZ(), mat.getRow(1).getZ() - mass*com.getY()*com.getZ());
    EXPECT_EQ(mat2.getRow(2).getZ(), mat.getRow(2).getZ() + mass*(com.getX()*com.getX() + com.getY() * com.getY()));

    tf::Matrix3x3 mat3 = jaco2KDL.getLinkInertiaCoM(linkName);
    EXPECT_EQ(mat3.getRow(0).getX(), mat.getRow(0).getX());
    EXPECT_EQ(mat3.getRow(0).getY(), mat.getRow(0).getY());
    EXPECT_EQ(mat3.getRow(0).getZ(), mat.getRow(0).getZ());
    EXPECT_EQ(mat3.getRow(1).getY(), mat.getRow(1).getY());
    EXPECT_EQ(mat3.getRow(1).getZ(), mat.getRow(1).getZ());
    EXPECT_EQ(mat3.getRow(2).getZ(), mat.getRow(2).getZ());

    com = tf::Vector3(1,2,3);
    std::vector<double> torques,t2;
    std::vector<double> qDot = {0.8, 0.1, 0.1, 0.1, 0.2, 0.1};
    torques.resize(6,0);
    std::vector<double> q2 = {4.776098185246001, 2.9779051856135985, 1.0370221453036228, -2.089493953881068, 1.3567380962770526, 1.3811624792663872};
    int  ec = jaco2KDL.getTorques(q2,qDot,qDot,torques);
    jaco2KDL.useUrdfDynamicParams();
    ec = jaco2KDL.getTorques(q2,qDot,qDot,t2);
    for(int i = 0; i <6; ++i)
    {
        std::cout << "t2(" << i <<") = " << t2[i] << " | torques(" << i <<") = " << torques[i] <<  std::endl;
        EXPECT_TRUE(t2[i] != torques[i]);
    }


}

TEST(Jaco2KinematicsDynamicsModelTest, IK)
{
    std::vector<double> zero;
    std::vector<double> jointAngles  = {0, M_PI, M_PI, 0, 0, M_PI};
    zero.resize(6,0);
    int nrTests = 1000;
    int fails = 0;
    for(int i = 0; i < nrTests; ++i){
        tf::Pose fk_pose;
        jaco2KDL.getRandomConfig(jointAngles);
        int ec = jaco2KDL.getFKPose(jointAngles,fk_pose,"jaco_link_hand");
        EXPECT_TRUE(ec>=0);
        if(ec>=0)
        {
            std::vector<double> ik_solution;
            int ecIK = jaco2KDL.getIKSolution(fk_pose,ik_solution,zero);
            if(ecIK < 0){
//                geometry_msgs::Pose msg;
//                tf::poseTFToMsg(fk_pose,msg);
//                std::cout << "number of test" << i<< std::endl << "Pose: " << msg << std::endl;

//                for(auto phi : jointAngles) {
//                    std::cout << phi << std::endl;
//                }
                ++fails;
            }
            //            EXPECT_TRUE(ecIK >= 0);
            if(ecIK >= 0){
                tf::Pose ik_pose;
                ecIK = jaco2KDL.getFKPose(ik_solution,ik_pose,"jaco_link_hand");
                EXPECT_NEAR(ik_pose.getOrigin().getX(), fk_pose.getOrigin().getX(), 1e-3);
                EXPECT_NEAR(ik_pose.getOrigin().getY(), fk_pose.getOrigin().getY(), 1e-3);
                EXPECT_NEAR(ik_pose.getOrigin().getZ(), fk_pose.getOrigin().getZ(), 1e-3);
                EXPECT_NEAR(ik_pose.getRotation().getX(), fk_pose.getRotation().getX(), 1e-3);
                EXPECT_NEAR(ik_pose.getRotation().getY(), fk_pose.getRotation().getY(), 1e-3);
                EXPECT_NEAR(ik_pose.getRotation().getZ(), fk_pose.getRotation().getZ(), 1e-3);
                EXPECT_NEAR(ik_pose.getRotation().getW(), fk_pose.getRotation().getW(), 1e-3);
            }
        }
    }
    double successRate = 1.0 - ((double)fails)/((double)nrTests);
    EXPECT_NEAR(successRate,0.99, 1e-2);
    std::cout << "success rate: " <<  successRate << std::endl;
}



int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_jaco2_dynamics");
    ros::NodeHandle node("~");
    std::string robot_desc_string;
    //    node.getParam("/robot_description", robot_desc_string);//, std::string(""));
    std::string urdf_param("robot_description");
    jaco2KDL = Jaco2KinematicsDynamicsModel(urdf_param,"jaco_link_base","jaco_link_hand");

    return RUN_ALL_TESTS();
}
