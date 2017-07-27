#include "static_data_generator.h"
#include "tree.hpp"
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>
using namespace jaco2_data;

StaticDataGenerator::StaticDataGenerator(ros::NodeHandle &nh):
    nh_(nh),
    buffer_length_(40),
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

    std::string bagName = nh_.param<std::string>("bag_name","/tmp/static_data.bag");
    bag_.open(bagName, rosbag::bagmode::Write);

}


void StaticDataGenerator::generateData(std::size_t depth)
{
    Node<n_joints, steps> tree;
//    int steps = 5;

    std::vector<std::vector<int>> index = Node<n_joints, steps>::getIndeces(tree);
    std::vector<double> goal(6,0);
    std::size_t n_points = index.size();
    std::size_t run = 0;
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
        ROS_INFO_STREAM("Progress: " << run/((float) n_points) * 100 << "%");
        ++run;

        saveStaticData();
    }

    bag_.close();

}

void StaticDataGenerator::saveStaticData()
{
    //TODO everything else
    ros::Duration d(1);
    ros::Rate r(30);
//    std::vector<
//    for()
}

void StaticDataGenerator::anglesCb(const jaco2_msgs::JointAnglesConstPtr& msg)
{
   JointAngles angles = jaco2_msgs::JointAngleConversion::ros2data(*msg);
   angle_buffer_.emplace_back(angles);
   while(angle_buffer_.size() > buffer_length_){
       angle_buffer_.pop_front();
   }
}

void StaticDataGenerator::stateCb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
{
    ExtendedJointStateData ex;
    ex.joint_state = jaco2_msgs::JointStateConversion::jaco2Msg2Data(*msg);
    ex.lin_acc = last_accs_;
    state_buffer_.emplace_back(ex);
    while(state_buffer_.size() > buffer_length_){
        state_buffer_.pop_front();
    }
}

void StaticDataGenerator::tauGfreeCb(const jaco2_msgs::Jaco2GfreeTorquesConstPtr& msg)
{
    JointData d;
    d.data = msg->effort_g_free;
    d.frame_id = msg->header.frame_id;
    d.stamp.fromNSec(msg->header.stamp.toNSec());
    tau_g_buffer_.emplace_back(d);
    while(tau_g_buffer_.size() > buffer_length_){
        tau_g_buffer_.pop_front();
    }
}

void StaticDataGenerator::tempCb(const jaco2_msgs::Jaco2SensorConstPtr& msg)
{
    JointData d;
    d.data = msg->temperature;
    d.stamp.fromNSec(msg->temperature_time.toNSec());
    temp_buffer_.emplace_back(d);
    while(temp_buffer_.size() > buffer_length_){
        temp_buffer_.pop_front();
    }
}

void StaticDataGenerator::exeCb(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
    status_buffer_.emplace_back(msg->status_list.front().status);
    while(status_buffer_.size() > buffer_length_){
        status_buffer_.pop_front();
    }
}

void StaticDataGenerator::accCb(const jaco2_msgs::Jaco2AccelerometersConstPtr& msg)
{
    last_accs_ = jaco2_msgs::AccelerometerConversion::ros2data(*msg);
}


