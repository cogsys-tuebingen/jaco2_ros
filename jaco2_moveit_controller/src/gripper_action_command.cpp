#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/GripperCommandAction.h>
#include <jaco2_msgs/GripperControlAction.h>

/// Moveit planning scene
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/solid_primitive_dims.h>

#if ROS_VERSION_MINIMUM(1,12,0)
#include <moveit/move_group_interface/move_group_interface.h>
typedef moveit::planning_interface::MoveGroupInterface MoveGroup;
#else
#include <moveit/move_group_interface/move_group.h>
typedef moveit::planning_interface::MoveGroup MoveGroup;
#endif

class GripperAction {

private:

    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;

    boost::scoped_ptr<actionlib::SimpleActionClient<jaco2_msgs::GripperControlAction>> gripper_ac_;
    boost::scoped_ptr<MoveGroup> gripper_;
    bool use_sim_;
    int finger_count_;


    // Variables

    double max_dist_ = 10.0;

    double max_angle_ = 0.8;
    double max_encoder_ = 7000;


    // Objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

public:

    GripperAction(std::string name) :
        as_(nh_, name, boost::bind(&GripperAction::executeCB, this, _1), false)
    {
        use_sim_ = nh_.param<bool>("use_sim", false);
        finger_count_ = nh_.param<int>("finger_count", 3);
        as_.start();

        if(use_sim_) {

            gripper_.reset(new MoveGroup("gripper"));
            ROS_INFO("Set up MoveGroup!");

        }
        else {

            gripper_ac_.reset(new actionlib::SimpleActionClient<jaco2_msgs::GripperControlAction>("jaco_22_driver/gripper_command", false));
            ROS_INFO("Set up GripperControl Actionserver!");

        }
    }

    ~GripperAction(void)
    {
        // as_.shutdown();
    }

    void executeCB(const control_msgs::GripperCommandGoalConstPtr &goal)
    {
        ROS_INFO("Received goal");

        // Possible interaction with position and max_effort goal message
        finger_count_ = nh_.param<int>("finger_count", 3);

        ROS_INFO("Grasping");

        /**
        SIMULATION PART
        **/

        if(use_sim_) {

            double angle = -max_angle_/max_dist_*goal->command.position + max_angle_;
            angle = min_value_max(0, angle, max_angle_);
            std::vector<double> config;

            if(finger_count_ == 2) {
                config = {angle, 0, angle, 0, 0, 0};
            }
            else {
                config = {angle, 0, angle, 0, angle, 0};
            }


            gripper_->setJointValueTarget(config);

            ROS_INFO("Set angle to %f", angle);

            MoveGroup::Plan my_plan;

            gripper_->setPlanningTime(5.0);
            bool result = static_cast<bool>(gripper_->plan(my_plan));

            if(result) {
                gripper_->execute(my_plan);
                ROS_INFO("Executed!");
                as_.setSucceeded();
            }
            else {
                ROS_INFO("Gripper wasn't able to move!");
                as_.setAborted();
            }

        }

        /**
        REAL PART
        **/

        else {
            gripper_ac_->waitForServer(ros::Duration(5.0));

            jaco2_msgs::GripperControlGoal gripper_goal;

            if(goal->command.position < 0) {

                if(finger_count_ == 2) {
                    gripper_goal.useFinger1 = true;
                    gripper_goal.useFinger2 = true;
                    gripper_goal.useFinger3 = false;
                    gripper_goal.usePos = true;
                    gripper_goal.posFinger3 = 0;
                }
                else {
                    gripper_goal.useFinger1 = true;
                    gripper_goal.useFinger2 = true;
                    gripper_goal.useFinger3 = true;
                }

            }
            else {

                double encode = -max_encoder_/max_dist_*goal->command.position + max_encoder_;
                encode = min_value_max(0, encode, max_encoder_);

                if(finger_count_ == 2) {

                    gripper_goal.useFinger1 = false;
                    gripper_goal.useFinger2 = false;
                    gripper_goal.useFinger3 = false;
                    gripper_goal.usePos = true;
                    gripper_goal.posFinger1 = encode;
                    gripper_goal.posFinger2 = encode;
                    gripper_goal.posFinger3 = 0;

                }
                else {
                    gripper_goal.useFinger1 = false;
                    gripper_goal.useFinger2 = false;
                    gripper_goal.useFinger3 = false;
                    gripper_goal.usePos = true;
                    gripper_goal.posFinger1 = encode;
                    gripper_goal.posFinger2 = encode;
                    gripper_goal.posFinger3 = encode;
                }
            }

            gripper_ac_->sendGoal(gripper_goal);

            bool success = gripper_ac_->waitForResult(ros::Duration(10.0));

            if(!success) {
                ROS_INFO("Gripper was not successfull before timeout!");
                as_.setAborted();
            }
            else {
                as_.setSucceeded();
            }

        }

    }



    double min_value_max(double min, double value, double max) {
        return std::max(std::min(max, value), min);
    }

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gripper_action_command");

    GripperAction gripperAction("gripper_action_command");

    ROS_INFO("Set up Server");

    ros::spin();

    return 0;

}
