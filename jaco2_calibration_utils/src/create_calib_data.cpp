#include <vector>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <jaco2_msgs/Jaco2Sensor.h>
//#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointVelocity.h>
#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/ArmJointAnglesAction.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>

std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
bool recieved_data_ = false;

void cb(const jaco2_msgs::Jaco2JointStateConstPtr& msg)
{
    Jaco2Calibration::DynamicCalibrationSample sample;
    for(std::size_t i = 0; i < 6; ++i){
        sample.jointPos[i] = msg->position[i];
        sample.jointVel[i] = msg->velocity[i];
        sample.jointTorque[i] = -msg->effort[i];
        sample.jointAcc[i] = msg->acceleration[i];
    }
    recieved_data_ = true;
    samples_.push_back(sample);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_create_calib_data_node");
    ros::NodeHandle private_nh_("~");
//    ros::Rate r(50);
    //    boost::function<void(const jaco2_msgs::Jaco2JointStateConstPtr&)> cb = boost::bind(&jointStateCb, this _1);
    ros::Subscriber subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_state_acc", 10, cb);
    ros::Publisher pubDone     = private_nh_.advertise<std_msgs::Bool>("create_data_done",2);
    actionlib::SimpleActionClient<jaco2_msgs::ArmJointAnglesAction> ac("/jaco_arm_driver/arm_joint_angles", true);
    ros::ServiceClient new_conf_client  = private_nh_.serviceClient<std_srvs::Trigger>("/dyn_calib_sampling/new_configuration");
    ros::ServiceClient save_conf_client = private_nh_.serviceClient<std_srvs::Trigger>("/dyn_calib_sampling/save_configurations");

    int iterations = 2;
    private_nh_.param<int>("iterations", iterations, 2);

    ac.waitForServer();
    ros::AsyncSpinner r(65);
    r.start();
    int i = 0;
    std_msgs::Bool done;
    done.data = false;
    pubDone.publish(done);

    ros::Duration d(2);
    while (ros::ok() && i < iterations) {


        ros::spinOnce();
        if(recieved_data_){
            double pos0 = samples_.back().jointPos[0];

            jaco2_msgs::ArmJointAnglesGoal goal;
            goal.type = jaco2_msgs::ArmJointAnglesGoal::RADIAN;
            goal.angles.joint1 = pos0 + 2*M_PI;
            goal.angles.joint2 = samples_.back().jointPos[1];
            goal.angles.joint3 = samples_.back().jointPos[2];
            goal.angles.joint4 = samples_.back().jointPos[3];
            goal.angles.joint5 = samples_.back().jointPos[4];
            goal.angles.joint6 = samples_.back().jointPos[5];

            ac.sendGoal(goal);

            bool finished = ac.waitForResult(ros::Duration(25));

            d.sleep();
            jaco2_msgs::ArmJointAnglesGoal goal1;
            goal1.type = jaco2_msgs::ArmJointAnglesGoal::RADIAN;
            goal1.angles.joint1 = pos0;
            goal1.angles.joint2 = goal.angles.joint2;
            goal1.angles.joint3 = goal.angles.joint3;
            goal1.angles.joint4 = goal.angles.joint4;
            goal1.angles.joint5 = goal.angles.joint5;
            goal1.angles.joint6 = goal.angles.joint6;

            ac.sendGoal(goal1);

            finished = ac.waitForResult(ros::Duration(25));

            std_srvs::Trigger tr;

            new_conf_client.call(tr.request, tr.response);
            save_conf_client.call(tr.request, tr.response);

            d.sleep();
            ++i;

//            r.sleep();
        }
        pubDone.publish(done);
        ros::spinOnce();

    }
    done.data = true;

    ros::Rate rd(10);
    for(std::size_t i = 0; i < 10; ++i){
        pubDone.publish(done);
        ros::spinOnce();
        rd.sleep();
    }

    return 0;
}
