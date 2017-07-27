#include "static_data_generator.h"
#include "tree.hpp"

StaticDataGenerator::StaticDataGenerator(ros::NodeHandle &nh):
    nh_(nh),
    depth_(1),
    run_(0),
    group_("manipulator")
{
    group_.setPlannerId("RRTkConfigDefault");
    group_.setStartStateToCurrentState();
    group_.setPlanningTime(5.0);

    std::string driver_name = nh_.param<std::string>("driver_name","jaco_22_driver");
    std::string prefix = "/" + driver_name + "/";

    sub_angles_ = nh_.subscribe(prefix + "out/joint_angles", 1, &StaticDataGenerator::anglesCb, this);
    sub_jaco_state_ = nh_.subscribe(prefix + "out/joint_state_acc", 1, &StaticDataGenerator::stateCb, this);
    sub_torque_gravity_ = nh_.subscribe(prefix + "out/torques_g_free", 1, &StaticDataGenerator::stateCb, this);
    sub_temp_ = nh_.subscribe(prefix + "out/sensor_info", 1, &StaticDataGenerator::tempCb, this);
    sub_execution_ = nh_.subscribe(prefix + "follow_joint_trajectory/manipulator/status", 1,&StaticDataGenerator::exeCb, this);
    sub_accs_ = nh_.subscribe(prefix + "out/accelerometers", 1, &StaticDataGenerator::accCb,this);

}


void StaticDataGenerator::generateData(std::size_t depth)
{
    Node<n_joints, steps> tree;
//    int steps = 5;

    std::vector<std::vector<int>> index = Node<n_joints, steps>::getIndeces(tree);
    std::vector<double> goal(6,0);
    for(auto id : index){
        for(std::size_t i = 0; i < 6; ++i){
            goal[i] = lower_limits_[i] + (double)id[i]*(upper_limits_[i] - lower_limits_[i])/double(steps);

        }
        for(int i = 0; i <5; ++i){
            group_.getCurrentState();
        }
        group_.setJointValueTarget(goal);
        moveit::planning_interface::MoveGroup::Plan my_plan;
        moveit_msgs::MoveItErrorCodes success = group_.plan(my_plan);
        if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Success! Now move");

            success = group_.move();
        }

        //TODO everything else
    }

}

void StaticDataGenerator::anglesCb(const jaco2_msgs::JointAnglesConstPtr& msg)
{

}

void StaticDataGenerator::stateCb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
{

}

void StaticDataGenerator::tauGfreeCb(const jaco2_msgs::Jaco2GfreeTorquesConstPtr& msg)
{

}

void StaticDataGenerator::tempCb(const jaco2_msgs::Jaco2SensorConstPtr& msg)
{

}

void StaticDataGenerator::exeCb(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{

}

void StaticDataGenerator::accCb(const jaco2_msgs::Jaco2AccelerometersConstPtr& msg)
{

}


