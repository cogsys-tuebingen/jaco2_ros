#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_kin_dyn_lib/joint_vel_pos_estimator.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

using namespace Jaco2KinDynLib;

std::vector<jaco2_msgs::Jaco2JointState> jdata = {};

std::vector<double> deriveVel(const std::vector<jaco2_msgs::Jaco2JointState>::iterator& it)
{
    auto prev = it -1;
    std::vector<double> vel;
    vel.resize(it->name.size());
    double dt = it->header.stamp.toSec() - prev->header.stamp.toSec();
    auto it_res = vel.begin();
    auto it_v_prev = prev->position.begin();
    for(auto v_next : it->position){
        *it_res = (v_next - *it_v_prev) / dt;
        ++it_res;
        ++it_v_prev;
    }
    return vel;
}

std::vector<double> deriveAcc(const std::vector<jaco2_msgs::Jaco2JointState>::iterator& it)
{
    auto prev = it -1;
    std::vector<double> acc;
    acc.resize(it->name.size());
    double dt = it->header.stamp.toSec() - prev->header.stamp.toSec();
    auto it_res = acc.begin();
    auto it_v_prev = prev->velocity.begin();
    for(auto v_next = it->velocity.begin(); v_next < it->velocity.end(); ++v_next){
        *it_res = (*v_next - *it_v_prev) / dt;
        ++it_res;
        ++it_v_prev;
    }

    return acc;
}

//TEST(PosVelIntegrationTest, integrationTest)
//{

//    JointVelPosIntegrator integrator("/robot_description","jaco_link_base","jaco_link_hand");

//    IntegrationData idata;
//    idata.time =jdata.front().header.stamp.toSec();
//    idata.vel = jdata.front().velocity;
//    idata.pos = jdata.front().position;
//    idata.torques = jdata.front().effort;
//    integrator.setInitalValues(idata);
//    integrator.integrate(idata);

//    rosbag::Bag outbag;
//    outbag.open("/tmp/fake_traj_test_simple_int.bag", rosbag::bagmode::Write);

//    Jaco2DynamicModel model("/robot_description","jaco_link_base","jaco_link_hand");
//    for(auto it = jdata.begin(); it < jdata.end(); ++it){
//        IntegrationData idata;
//        idata.time =it->header.stamp.toSec();
//        idata.torques = it->effort;
//        idata.vel = it->velocity;
//        idata.pos = it->position;

//        integrator.integrate(idata);

//        outbag.write("/jaco_joint_states", it->header.stamp, *it);
//        jaco2_msgs::Jaco2JointState j2 = *it;
//        j2.position = integrator.getCurrentPosition();
//        j2.velocity = integrator.getCurrentVelocity();
//        j2.acceleration = integrator.getCurrentAcceleration();
//        model.getTorques(it->position, it->velocity, it->acceleration, j2.effort);


//        for(std::size_t i = 0; i < 6; ++i){

//            //                EXPECT_NEAR(j2.velocity[i], it_test->velocity[i],0.3);
//            EXPECT_NEAR(j2.position[i], it->position[i],0.04);
//        }
//        outbag.write("/integrated_state", j2.header.stamp, j2);
//    }

//    outbag.close();
//}

TEST(PosVelIntegrationTest, simpleEstimateTest)
{

    JointVelPosEstimator integrator("/robot_description","jaco_link_base","jaco_link_hand");

    IntegrationData idata;
    idata.dt = 0;
    idata.vel = jdata.front().velocity;
    idata.pos = jdata.front().position;
    idata.torques = jdata.front().effort;
    integrator.setInitalValues(idata);
//    integrator.estimate(idata);

    rosbag::Bag outbag;
    outbag.open("/tmp/fake_traj_test_simple_est.bag", rosbag::bagmode::Write);

    Jaco2DynamicModel model("/robot_description","jaco_link_base","jaco_link_hand");
    for(auto it = jdata.begin(); it < jdata.end(); ++it){
        IntegrationData idata;
        idata.torques = it->effort;
        idata.vel = it->velocity;
        idata.pos = it->position;
        if(it> jdata.begin()){
            auto it_prev = it -1;
            auto dt = it->header.stamp - it_prev->header.stamp;
            idata.dt = dt.toSec();
        }
        else{
            idata.dt = 0;
        }

        integrator.estimate(idata);

        outbag.write("/jaco_joint_states", it->header.stamp, *it);
        jaco2_msgs::Jaco2JointState j2 = *it;
        j2.position = integrator.getCurrentPosition();
        j2.velocity = integrator.getCurrentVelocity();
        j2.acceleration = integrator.getCurrentAcceleration();
        model.getTorques(it->position, it->velocity, it->acceleration, j2.effort);
        ros::Duration d(integrator.getTime());

        j2.header.stamp = jdata.front().header.stamp +d;
        if(it < jdata.end() -1 ){
            for(std::size_t i = 0; i < 6; ++i){
                auto it_test = it +1;
                //                EXPECT_NEAR(j2.velocity[i], it_test->velocity[i],0.3);
                EXPECT_NEAR(j2.position[i], it_test->position[i],0.04);

            }
        }
        outbag.write("/integrated_state", j2.header.stamp, j2);
    }

    outbag.close();
}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_integration");

    std::string path = argv[1];
    rosbag::Bag bag;
    bag.open(path);

    std::vector<std::string> topics;
    //    topics.push_back(std::string("/joint_states"));
    topics.push_back(std::string("/jaco_22_driver/out/joint_states"));


    //fetch data
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(auto m : view){
        sensor_msgs::JointStateConstPtr s = m.instantiate<sensor_msgs::JointState>();
        jaco2_msgs::Jaco2JointState state;
        state.header = s->header;
        state.name.insert(state.name.end(), s->name.begin(), s->name.begin() +6);
        state.position.insert(state.position.end(), s->position.begin(), s->position.begin() +6);
        state.velocity.resize(6,0);
        state.effort.resize(6,0);
        //        state.velocity.insert(state.velocity.end(), s->velocity.begin(), s->velocity.begin() +6);
        //        state.effort.insert(state.effort.end(), s->effort.begin(), s->effort.begin() +6);
        state.acceleration.resize(6,0);
        jdata.emplace_back(state);
    }

    // derive missing data
    Jaco2DynamicModel model("/robot_description","jaco_link_base","jaco_link_hand");

    for(auto it = jdata.begin(); it < jdata.end(); ++it){
        if(it > jdata.begin()){
            //            std::vector<double> vel = deriveVel(it);
            //            it->velocity = vel;
            std::vector<double> acc = deriveAcc(it);
            it->acceleration = acc;
        }

        std::vector<double> torques;
        int ec = model.getTorques(it->position, it->velocity, it->acceleration, torques);
        it->effort = torques;

    }

    bag.close();


    return RUN_ALL_TESTS();
}
