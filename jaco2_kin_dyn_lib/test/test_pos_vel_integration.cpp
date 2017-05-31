#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <jaco2_kin_dyn_lib/jaco2_dynamic_model.h>
#include <jaco2_kin_dyn_lib/joint_vel_pos_integrator.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <ros/ros.h>
using namespace Jaco2KinDynLib;


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
}

std::vector<double> deriveAcc(const std::vector<jaco2_msgs::Jaco2JointState>::iterator& it)
{
    auto prev = it -1;
    std::vector<double> vel;
    vel.resize(it->name.size());
    double dt = it->header.stamp.toSec() - prev->header.stamp.toSec();
    auto it_res = vel.begin();
    auto it_v_prev = prev->velocity.begin();
    for(auto v_next : it->velocity){
        *it_res = (v_next - *it_v_prev) / dt;
        ++it_res;
        ++it_v_prev;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_integration");

    std::string path = argv[1];
    rosbag::Bag bag;
    bag.open(path);

    std::vector<std::string> topics;
    topics.push_back(std::string("/joint_states"));


    //fetch data
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    std::vector<jaco2_msgs::Jaco2JointState> jdata;
    for(auto m : view){
        sensor_msgs::JointStateConstPtr s = m.instantiate<sensor_msgs::JointState>();
        jaco2_msgs::Jaco2JointState state;
        state.header = s->header;
        state.name.insert(state.name.end(), s->name.begin(), s->name.begin() +6);
        state.position.insert(state.position.end(), s->position.begin(), s->position.begin() +6);
        state.velocity.resize(6,0);
        state.effort.resize(6,0);
        state.acceleration.resize(6,0);
//        exstate.state.header.stamp.toSec()
    }

    // derive missing data
    Jaco2DynamicModel model("/robot_description","jaco_link_base","jaco_link_hand");
    JointVelPosIntegrator integrator("/robot_description","jaco_link_base","jaco_link_hand");
    rosbag::Bag outbag;
    bag.open("/tmp/fake_test_traj.bag", rosbag::bagmode::Write);

    integrator.setInitalValues(jdata.front().position, jdata.front().velocity);

    std::size_t counter;
    for(auto it = jdata.begin(); it < jdata.end(); ++it){
        if(it > jdata.begin()){
            it->velocity = deriveVel(it);
            it->acceleration = deriveAcc(it);
        }
        std::vector<double> torques;
        model.getTorques(it->position, it->velocity, it->acceleration, torques);
        it->effort = torques;


        IntegrationData idata;
        idata.time =it->header.stamp.toSec();
        idata.torques = torques;

        integrator.integrate(idata);


        if(counter >= 3){

            outbag.write("/jaco_joint_states", it->header.stamp, *it);
            jaco2_msgs::Jaco2JointState j2 = *it;
            j2.position = integrator.getCurrentPosition();
            j2.velocity = integrator.getCurrentVelocity();
            j2.acceleration = integrator.getCurrentAcceleration();
            outbag.write("/integrated_state", it->header.stamp, j2);
        }

        ++counter;
    }

    bag.close();
    outbag.close();


    return 0;
}
