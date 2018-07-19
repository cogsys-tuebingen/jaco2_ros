#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/GripperCommandAction.h>
#include <jaco2_msgs/GripperControlAction.h>

/// Moveit planning scene
#include <moveit_msgs/PlanningScene.h>
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
    moveit_msgs::PlanningScene planning_scene;
    ros::Publisher planning_scene_diff_publisher_;

public:

    GripperAction(std::string name) :
        as_(nh_, name, boost::bind(&GripperAction::executeCB, this, _1), false)
    {
        use_sim_ = nh_.param<bool>("use_sim", false);
        finger_count_ = nh_.param<int>("finger_count", 3);
        as_.start();

        // Setup planning scene
        planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

        if(use_sim_) {

            gripper_.reset(new MoveGroup("gripper"));
            ROS_INFO("Set up MoveGroup!");

        }
        else {

            gripper_ac_.reset(new actionlib::SimpleActionClient<jaco2_msgs::GripperControlAction>("gripper_server", false));
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
                objectManagement(goal->command.position);
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
                objectManagement(goal->command.position);
            }

        }

    }

    void objectManagement(double object_size_){

        if(object_size_ < 10.0) {

            moveit_msgs::AttachedCollisionObject attached_object;
            attached_object.link_name = "jaco_link_hand";
            attached_object.object.header.frame_id = "jaco_link_hand";
            attached_object.object.id = "box";

            /* A default pose */
            geometry_msgs::Pose pose;
            pose.position.z = -0.2;
            pose.orientation.w = 1.0;

            /* Define a box to be attached */
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = object_size_*10e-3;
            primitive.dimensions[1] = object_size_*10e-3;
            primitive.dimensions[2] = object_size_*10e-3;

            attached_object.object.primitives.push_back(primitive);
            attached_object.object.primitive_poses.push_back(pose);

            attached_object.object.operation = attached_object.object.ADD;

            ROS_INFO("Adding the object into the world at the location of the hand.");

            planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
            planning_scene.is_diff = true;
            planning_scene_diff_publisher_.publish(planning_scene);

        }
        else {

            /* First, define the DETACH object message*/
            moveit_msgs::AttachedCollisionObject detach_object;
            detach_object.object.id = "box";
            detach_object.link_name = "jaco_link_hand";
            detach_object.object.operation = detach_object.object.REMOVE;

            /* Carry out the DETACH + ADD operation */
            ROS_INFO("Detaching the object from the robot and returning it to the world.");

            planning_scene.robot_state.attached_collision_objects.clear();
            planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
            planning_scene.robot_state.is_diff = true;
            planning_scene.world.collision_objects.clear();
            planning_scene.world.collision_objects.push_back(detach_object.object);
            planning_scene.is_diff = true;
            planning_scene_diff_publisher_.publish(planning_scene);

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
