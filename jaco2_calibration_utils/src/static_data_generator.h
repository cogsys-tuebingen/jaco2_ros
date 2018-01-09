#ifndef STATIC_DATA_GENERATOR_H
#define STATIC_DATA_GENERATOR_H
/// SYSTEM
#include <deque>
#include <mutex>
/// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
/// JACO2
#include <jaco2_data/extended_joint_state_data.h>
#include <jaco2_data/joint_angles.h>
#include <jaco2_data/joint_data.h>
#include <jaco2_msgs/JointAngles.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/Jaco2GfreeTorques.h>
#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/Jaco2Accelerometers.h>
#include <jaco2_data/types.h>

class StaticDataGenerator
{
public:
    StaticDataGenerator(ros::NodeHandle& nh);
    ~StaticDataGenerator();
    void generateData();

    void setUpperLimit(int id, double val){upper_limits_[id] = val;}
    void setLowerLimit(int id, double val){lower_limits_[id] = val;}

    void saveBag();

private:
    void anglesCb(const jaco2_msgs::JointAnglesConstPtr& msg);
    void stateCb(const jaco2_msgs::Jaco2JointStateConstPtr& msg);
    void tauGfreeCb(const jaco2_msgs::Jaco2GfreeTorquesConstPtr& msg);
    void tempCb(const jaco2_msgs::Jaco2SensorConstPtr& msg);
    void exeCb(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
    void accCb(const jaco2_msgs::Jaco2AccelerometersConstPtr& msg);

    void saveStaticData();
    bool moving();
    std::size_t searchStart() const;
    bool discreteVectorEqual(const std::vector<int> first,
                             const std::vector<int>& second) const;

private:
    ros::NodeHandle nh_;
    std::size_t buffer_length_;
    std::size_t n_steps_;
    std::size_t n_joints_;
    std::vector<double> upper_limits_;
    std::vector<double> lower_limits_;
    std::size_t valid_counter_;
    moveit::planning_interface::MoveGroupInterface group_;
    ros::Subscriber sub_angles_;
    ros::Subscriber sub_jaco_state_;
    ros::Subscriber sub_torque_gravity_;
    ros::Subscriber sub_temp_;
    ros::Subscriber sub_execution_;
    ros::Subscriber sub_accs_;
    rosbag::Bag bag_;
    mutable std::recursive_mutex data_mutex_;
    std::vector<std::vector<int>> steps_;

    jaco2_data::ExtendedJointStateDeque state_buffer_;
    std::deque<jaco2_data::JointAngles> angle_buffer_;
    std::deque<jaco2_data::JointDataStamped> tau_g_buffer_;
    std::deque<jaco2_data::JointDataStamped> temp_buffer_;
    std::deque<int> status_buffer_;

    jaco2_data::AccelerometerData last_accs_;
    std::vector<int> start_vec_;
};

#endif // STATIC_DATA_GENERATOR_H
