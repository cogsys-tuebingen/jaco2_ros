#include "static_data_generator.h"
#include "tree.hpp"
#include <jaco2_msgs_conversion/jaco2_ros_msg_conversion.h>
using namespace jaco2_data;

StaticDataGenerator::StaticDataGenerator(ros::NodeHandle &nh):
    nh_(nh),
    buffer_length_(40),
    upper_limits_(n_joints),
    lower_limits_(n_joints),
    group_("manipulator")
{
    group_.setPlannerId("RRTkConfigDefault");
    group_.setStartStateToCurrentState();
    group_.setPlanningTime(5.0);

    std::string driver_name = nh_.param<std::string>("driver_name","jaco_22_driver");
    std::string prefix = "/" + driver_name + "/";

    sub_angles_ = nh_.subscribe(prefix + "out/joint_angles", 1, &StaticDataGenerator::anglesCb, this);
    sub_jaco_state_ = nh_.subscribe(prefix + "out/joint_state_acc", 1, &StaticDataGenerator::stateCb, this);
    sub_torque_gravity_ = nh_.subscribe(prefix + "out/torques_g_free", 1, &StaticDataGenerator::tauGfreeCb, this);
    sub_temp_ = nh_.subscribe(prefix + "out/sensor_info", 1, &StaticDataGenerator::tempCb, this);
    sub_execution_ = nh_.subscribe(prefix + "follow_joint_trajectory/manipulator/status", 1,&StaticDataGenerator::exeCb, this);
    sub_accs_ = nh_.subscribe(prefix + "out/accelerometers", 1, &StaticDataGenerator::accCb,this);


    nh.param<double>("joint_1_upper_limit",upper_limits_[0],4.808 + 0.3);
    nh.param<double>("joint_1_lower_limit",lower_limits_[0],4.808 - 0.3);

    nh.param<double>("joint_2_upper_limit",upper_limits_[1], 2.96 + 0.3);
    nh.param<double>("joint_2_lower_limit",lower_limits_[1], 2.96 - 0.3);

    nh.param<double>("joint_3_upper_limit",upper_limits_[2], 1.00 + 0.3);
    nh.param<double>("joint_3_lower_limit",lower_limits_[2], 1.00 - 0.3);

    nh.param<double>("joint_4_upper_limit",upper_limits_[3], 4.20 + 0.3);
    nh.param<double>("joint_4_lower_limit",lower_limits_[3], 4.20 - 0.3);

    nh.param<double>("joint_5_upper_limit",upper_limits_[4], 1.45 + 0.3);
    nh.param<double>("joint_5_lower_limit",lower_limits_[4], 1.45 - 0.3);

    nh.param<double>("joint_6_upper_limit",upper_limits_[5], 1.32 + 0.3);
    nh.param<double>("joint_6_lower_limit",lower_limits_[5], 1.32 - 0.3);

    std::string bagName = nh_.param<std::string>("bag_name","/tmp/static_data.bag");
    bag_.open(bagName, rosbag::bagmode::Write);

}


void StaticDataGenerator::generateData()
{
    Node<n_joints, steps> tree;
//    int steps = 5;

    std::vector<std::vector<int>> index = Node<n_joints, steps>::getIndeces(tree);
    std::vector<double> goal(6,0);
    std::size_t n_points = index.size();
    std::size_t run = 0;
    ros::Time start = ros::Time::now();
    for(auto id : index){
        std::string cfg;
        for(auto i : id){
            cfg += std::to_string(i) + ",";
        }
        cfg.pop_back();
        ROS_INFO_STREAM("Config: " << cfg);
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
        ros::Duration dur = ros::Time::now() - start;

        ROS_INFO_STREAM("Progress: " << run/((float) n_points) * 100 << "% | Steps: ("
                        << run << "/ "
                        << index.size() << ") | time since start (h:min:sec): "
                        << (int)dur.toSec()/3600 << ":"
                        << ((int)dur.toSec()/60)  % 60 << ":"
                        << ((int)dur.toSec()) % 60);


        ++run;

        saveStaticData();
    }

    bag_.close();

}

void StaticDataGenerator::saveStaticData()
{
    ros::Duration(0.5).sleep();
    ros::Duration d(1);
    ros::Rate r(30);
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < d){
        ros::spinOnce();
        r.sleep();
    }
    auto it_state = state_buffer_.begin();
    auto it_angle = angle_buffer_.begin();
    auto it_tmp = temp_buffer_.begin();
    auto it_tau = tau_g_buffer_.begin();
    std::size_t state_counter = 0;
    std::size_t angle_counter = 0;
    std::size_t temp_counter = 0;
    std::size_t tau_counter = 0;
    ExtendedJointStateData state_m(n_joints, n_joints);
    JointAngles angle_m(n_joints);
    JointData temp_m(n_joints);
    JointData tau_m(n_joints);
    for(auto v : status_buffer_)
    {
        if(v == actionlib_msgs::GoalStatus::SUCCEEDED){
            if(it_state < state_buffer_.end()){
                state_m += *it_state;
                ++state_counter;
            }
            if(it_angle < angle_buffer_.end()){
                angle_m += *it_angle;
                ++it_angle;
                ++angle_counter;
            }
            if(it_tmp < temp_buffer_.end()){
                temp_m += *it_tmp;
                ++it_tmp;
                ++temp_counter;
            }
            if(it_tau < tau_g_buffer_.end()){
                tau_m += *it_tau;
                ++it_tau;
                ++tau_counter;
            }
        }
    }
    if(state_counter > 0){
        state_m /= (double) state_counter;
    }
    else{
        ROS_WARN("Did not get any JointState data!");
    }
    if(angle_counter > 0){
        angle_m /= (double) angle_counter;
    }
    else{
        ROS_WARN("Did not get any JointAngle data!");
    }
    if(temp_counter > 0){
        temp_m /= (double) temp_counter;
    }
    else{
        ROS_WARN("Did not get any JointState data!");
    }
    if(tau_counter > 0){
        tau_m /= (double) tau_counter;
    }
    else{
        ROS_WARN("Did not get any TorqueGFree data!");
    }
    ros::Time wtime = ros::Time::now();
    bag_.write("/joint_state", wtime, jaco2_msgs::JointStateConversion::datata2Jaco2Msgs(state_m.joint_state));
    bag_.write("/acceleration", wtime, jaco2_msgs::AccelerometerConversion::data2ros(state_m.lin_acc));
    bag_.write("/angles", wtime, jaco2_msgs::JointAngleConversion::data2ros(angle_m));
    bag_.write("/torques_g_free", wtime, jaco2_msgs::JointDataConversion::data2ros(tau_m));
    bag_.write("/temperature", wtime, jaco2_msgs::JointDataConversion::data2ros(temp_m));
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

void StaticDataGenerator::saveBag()
{
    bag_.close();
}
