#include <string>
#include <vector>
#include <math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <jaco2_kin_dyn_lib/jaco2_kinematics_dynamics.h>


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
    EXPECT_EQ(jaco2KDL.getKDLSegmentIndex("jaco_link_base"), -1);
    EXPECT_EQ(jaco2KDL.getKDLSegmentIndex("jaco_link_1"),0);
    EXPECT_EQ(jaco2KDL.getKDLSegmentIndex("jaco_link_5"),4);
    EXPECT_EQ(jaco2KDL.getKDLSegmentIndex("jaco_link_hand"), 5);
    EXPECT_EQ(jaco2KDL.getKDLSegmentIndexFK("jaco_link_5"), 5);
}

TEST(Jaco2KinematicsDynamicsModelTest, DynParam)
{
    Eigen::Vector3d comLink1 = jaco2KDL.getLinkCoM("jaco_link_1");
    EXPECT_NEAR(comLink1(0), 0, 1e-4);
    EXPECT_NEAR(comLink1(1),  0.0086, 1e-4);
    EXPECT_NEAR(comLink1(2), -0.1064, 1e-4);
    EXPECT_NEAR(jaco2KDL.getLinkMass("jaco_link_1"),0.7417, 1e-4);
    Eigen::Matrix3d inertia = jaco2KDL.getLinkInertia("jaco_link_1");
    EXPECT_NEAR(inertia(0,0), 0.0094, 1e-4);
    EXPECT_NEAR(inertia(0,1), 0.0000, 1e-4);
    EXPECT_NEAR(inertia(0,2), 0.0000, 1e-4);
    EXPECT_NEAR(inertia(1,1), 0.0522, 1e-4);
    EXPECT_NEAR(inertia(1,2), 0.0006, 1e-4);
    EXPECT_NEAR(inertia(2,2), 0.0004, 1e-4);

    Eigen::Vector3d comLink3 = jaco2KDL.getLinkCoM("jaco_link_3");
    EXPECT_NEAR(comLink3(0),  0.1624, 1e-4);
    EXPECT_NEAR(comLink3(1),  0.0000, 1e-4);
    EXPECT_NEAR(comLink3(2), -0.0135, 1e-4);
    EXPECT_NEAR(jaco2KDL.getLinkMass("jaco_link_3"), 0.7847, 1e-4);
    inertia = jaco2KDL.getLinkInertia("jaco_link_3");
    EXPECT_NEAR(inertia(0,0),  0.0004, 1e-4);
    EXPECT_NEAR(inertia(0,1),  0.0000, 1e-4);
    EXPECT_NEAR(inertia(0,2),  0.0015, 1e-4);
    EXPECT_NEAR(inertia(1,1),  0.0243, 1e-4);
    EXPECT_NEAR(inertia(1,2),  0.0000, 1e-4);
    EXPECT_NEAR(inertia(2,2),  0.0243, 1e-4);

}

TEST(Jaco2KinematicsDynamicsModelTEST, KinParam)
{
    Eigen::Vector3d trans = jaco2KDL.getLinkFixedTranslation("jaco_link_1");
    Eigen::Matrix3d rot = jaco2KDL.getLinkFixedRotation("jaco_link_1");
    EXPECT_NEAR(trans(0), 0, 1e-4);
    EXPECT_NEAR(trans(1), 0, 1e-4);
    EXPECT_NEAR(trans(2), 0.1535, 1e-4);
    EXPECT_NEAR(rot(0,0),  1, 1e-4);
    EXPECT_NEAR(rot(0,1),  0, 1e-4);
    EXPECT_NEAR(rot(0,2),  0, 1e-4);
    EXPECT_NEAR(rot(1,0),  0, 1e-4);
    EXPECT_NEAR(rot(1,1), -1, 1e-4);
    EXPECT_NEAR(rot(1,2),  0, 1e-4);
    EXPECT_NEAR(rot(2,0),  0, 1e-4);
    EXPECT_NEAR(rot(2,1),  0, 1e-4);
    EXPECT_NEAR(rot(2,2), -1, 1e-4);
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
    Eigen::Vector3d com(1.1, 0.1, 0.1);
    Eigen::Matrix3d mat;
    mat << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;
    jaco2KDL.changeDynamicParams(linkName,jaco2KDL.getLinkMass(linkName), com, mat);
    double mass = jaco2KDL.getLinkMass(linkName);
    Eigen::Vector3d v = jaco2KDL.getLinkCoM(linkName);
    Eigen::Matrix3d mat2 = jaco2KDL.getLinkInertia(linkName);
    EXPECT_NEAR(v(0),com(0), 1e-6);
    EXPECT_NEAR(v(1),com(1), 1e-6);
    EXPECT_NEAR(v(2),com(2), 1e-6);
    EXPECT_EQ(mat2(0,0), mat(0,0) + mass*(com(1)*com(1) + com(2) * com(2)));
    EXPECT_EQ(mat2(0,1), mat(0,1) - mass*com(0)*com(1));
    EXPECT_EQ(mat2(0,2), mat(0,2) - mass*com(0)*com(2));
    EXPECT_EQ(mat2(1,1), mat(1,1) + mass*(com(0)*com(0) + com(2) * com(2)));
    EXPECT_EQ(mat2(1,2), mat(1,2) - mass*com(1)*com(2));
    EXPECT_EQ(mat2(2,2), mat(2,2) + mass*(com(0)*com(0) + com(1) * com(1)));

    Eigen::Matrix3d mat3 = jaco2KDL.getLinkInertiaCoM(linkName);
    EXPECT_EQ(mat3(0,0), mat(0,0));
    EXPECT_EQ(mat3(0,1), mat(0,1));
    EXPECT_EQ(mat3(0,2), mat(0,2));
    EXPECT_EQ(mat3(1,1), mat(1,1));
    EXPECT_EQ(mat3(1,2), mat(1,2));
    EXPECT_EQ(mat3(2,2), mat(2,2));

    com = Eigen::Vector3d(1,2,3);
    std::vector<double> torques,t2;
    std::vector<double> qDot = {0.8, 0.1, 0.1, 0.1, 0.2, 0.1};
    torques.resize(6,0);
    std::vector<double> q2 = {4.776098185246001, 2.9779051856135985, 1.0370221453036228, -2.089493953881068, 1.3567380962770526, 1.3811624792663872};
    int  ec = jaco2KDL.getTorques(q2,qDot,qDot,torques);
    jaco2KDL.useUrdfDynamicParams();
    ec = jaco2KDL.getTorques(q2,qDot,qDot,t2);
    for(int i = 0; i <6; ++i)
    {
//        std::cout << "t2(" << i <<") = " << t2[i] << " | torques(" << i <<") = " << torques[i] <<  std::endl;
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

TEST(Jaco2KinematicsDynamicsModelTest, kdlEigenConversion)
{
    KDL::Vector vec1(1,2,3);
    KDL::Vector vec2(3,4,5);
    KDL::Vector kdlcross = vec1*vec2;

    Eigen::Vector3d vec2_eigen(3,4,5);
    Eigen::Matrix3d mat = Jaco2KinematicsDynamicsModel::skewSymMat(vec1);
    Eigen::Vector3d crossprod = mat * vec2_eigen;

    KDL::Rotation r(0.707107,0.707107,0,
                    0,0,1,
                    0.707107,-0.707107,0);

    KDL::Vector rot_vec2 = r * vec2;

    Eigen::Vector3d rot_vec2_e= Jaco2KinematicsDynamicsModel::kdlMatrix2Eigen(r) * vec2_eigen;

    KDL::Vector t(0.1,0.2,0.3);
    KDL::Frame frame(r,t);
    KDL::Twist twist(vec1, vec2);
    KDL::Twist twist_trans = frame*twist;

    Eigen::Matrix<double, 6, 1> vec1_spatial;
    vec1_spatial << vec2(0), vec2(1), vec2(2), vec1(0), vec1(1), vec1(2);
    Eigen::Matrix<double, 6, 6> eframe = Jaco2KinematicsDynamicsModel::kdlFrame2Spatial(frame);
    Eigen::Matrix<double, 6, 1> etrans = eframe *vec1_spatial;

    KDL::Wrench w_t = frame * KDL::Wrench(vec1,vec2);
    Eigen::Matrix<double, 6, 1> wrench;
    wrench << vec2(0), vec2(1), vec2(2), vec1(0), vec1(1), vec1(2);
    KDL::Frame i_frame = frame.Inverse();
    Eigen::Matrix<double, 6, 1> wrench_t = Jaco2KinematicsDynamicsModel::kdlFrame2Spatial(i_frame).transpose() * wrench;

    Eigen::Matrix<double, 3, 6> in_prod =Jaco2KinematicsDynamicsModel::inertiaProductMat(KDL::Vector(1,2,3));


    for(int i = 0; i < 3; ++i) {
        EXPECT_EQ(kdlcross(i), crossprod(i));
        EXPECT_EQ(rot_vec2(i), rot_vec2_e(i));
        EXPECT_NEAR(twist_trans.rot(i), etrans(i), 1e-6);
        EXPECT_NEAR(twist_trans.vel(i), etrans(i+3), 1e-6);
        EXPECT_NEAR(w_t.torque(i), wrench_t(i), 1e-6);
        EXPECT_NEAR(w_t.force(i), wrench_t(i+3), 1e-6);
        EXPECT_EQ(in_prod(0,i+3),0);
    }
    EXPECT_EQ(in_prod(0,0),1);
    EXPECT_EQ(in_prod(0,1),2);
    EXPECT_EQ(in_prod(0,2),3);
    EXPECT_EQ(in_prod(1,0),0);
    EXPECT_EQ(in_prod(1,1),1);
    EXPECT_EQ(in_prod(1,2),0);
    EXPECT_EQ(in_prod(1,3),2);
    EXPECT_EQ(in_prod(1,4),3);
    EXPECT_EQ(in_prod(1,5),0);
    EXPECT_EQ(in_prod(2,0),0);
    EXPECT_EQ(in_prod(2,1),0);
    EXPECT_EQ(in_prod(2,2),1);
    EXPECT_EQ(in_prod(2,3),0);
    EXPECT_EQ(in_prod(2,4),2);
    EXPECT_EQ(in_prod(2,5),3);
}


TEST(Jaco2KinematicsDynamicsModelTest, getRigidBodyRegressionMatrix)
{
//    std::vector<double> q = {4.77, 3.14, 3.14, 0.0, 0, 3.14};
    std::vector<double> q = {4.77, 2.97, 1.03, -2.08, 1.36, 1.38};
//    std::vector<double> qDot = {0.1, 0.04, 0.05, 0.06, 0.03, 0.1};
//    std::vector<double> qDotDot = {0.1, 0, 0, 0, -0.1, 0.4};
    std::vector<double> qDot = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> qDotDot = {0.0, 0, 0, 0, 0.0, 0.0};

    std::vector<double> torque;
    jaco2KDL.getTorques(q, qDot, qDotDot, torque);
    Eigen::Matrix<double, 60, 1> param_vec;
    int id = 0;
    for(auto link : jaco2KDL.getLinkNames()) {
        param_vec(id) = jaco2KDL.getLinkMass(link);
        Eigen::Vector3d mc = jaco2KDL.getLinkMass(link) * jaco2KDL.getLinkCoM(link);
        param_vec(id+1) = mc(0);
        param_vec(id+2) = mc(1);
        param_vec(id+3) = mc(2);
        param_vec(id+4) = jaco2KDL.getLinkInertia(link)(0,0);
        param_vec(id+5) = jaco2KDL.getLinkInertia(link)(0,1);
        param_vec(id+6) = jaco2KDL.getLinkInertia(link)(0,2);
        param_vec(id+7) = jaco2KDL.getLinkInertia(link)(1,1);
        param_vec(id+8) = jaco2KDL.getLinkInertia(link)(1,2);
        param_vec(id+9) = jaco2KDL.getLinkInertia(link)(2,2);
        id += 10;
    }
    Eigen::Matrix<double, 10,1> param_vec6 = Eigen::Matrix<double, 10, 1>::Zero();
    param_vec6 = param_vec.block<10,1>(50,0);

    Eigen::Matrix<double, 1, 10> matrix = jaco2KDL.getRigidBodyRegressionMatrix("jaco_link_hand", "jaco_link_hand",q, qDot, qDotDot);
    double tau6 = matrix*param_vec6;
    EXPECT_NEAR(torque[5], tau6, 1e-5);

    Eigen::Matrix<double, 2, 20> mat56 = jaco2KDL.getRigidBodyRegressionMatrix("jaco_link_5", "jaco_link_hand",q, qDot, qDotDot);
    Eigen::Matrix<double, 2, 1> tau56 = mat56*param_vec.block<20,1>(40,0);
    EXPECT_NEAR(torque[4], tau56(0), 1e-5);
    EXPECT_NEAR(torque[5], tau56(1), 1e-5);

    Eigen::Matrix<double, 6, 60> full_matrix = jaco2KDL.getRigidBodyRegressionMatrix("jaco_link_1", "jaco_link_hand",q, qDot, qDotDot);
    Eigen::Matrix<double, 6, 1> tau_full = full_matrix*param_vec;

    for(int i = 0; i <6; ++i) {
        EXPECT_NEAR(torque[i], tau_full(i), 1e-5);
    }

    qDot = {0.1, 0.04, 0.05, 0.06, 0.03, 0.1};
    jaco2KDL.getTorques(q, qDot, qDotDot, torque);

    matrix = jaco2KDL.getRigidBodyRegressionMatrix("jaco_link_hand", "jaco_link_hand",q, qDot, qDotDot);
    tau6 = matrix*param_vec6;
    EXPECT_NEAR(torque[5], tau6, 1e-5);

    mat56 = jaco2KDL.getRigidBodyRegressionMatrix("jaco_link_5", "jaco_link_hand",q, qDot, qDotDot);
    tau56 = mat56*param_vec.block<20,1>(40,0);
    EXPECT_NEAR(torque[4], tau56(0), 1e-4);
    EXPECT_NEAR(torque[5], tau56(1), 1e-4);

    full_matrix = jaco2KDL.getRigidBodyRegressionMatrix("jaco_link_1", "jaco_link_hand",q, qDot, qDotDot);
    tau_full = full_matrix*param_vec;

    for(int i = 0; i <6; ++i) {
        EXPECT_NEAR(torque[i], tau_full(i), 1e-5);
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
    jaco2KDL = Jaco2KinematicsDynamicsModel(urdf_param,"jaco_link_base","jaco_link_hand");

    return RUN_ALL_TESTS();
}
