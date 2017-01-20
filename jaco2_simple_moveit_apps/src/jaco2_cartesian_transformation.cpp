#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/conversions.h>
#include <jaco2_msgs/CartesianTransformation.h>
#include <tf/tf.h>
#include <ros/ros.h>


class CartesianTransformationServer{
public:
    CartesianTransformationServer(std::string group_name, ros::NodeHandle& nh):
        group_(group_name)
    {
        fk_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

        move_cart_server_ = nh.advertiseService<jaco2_msgs::CartesianTransformation>("cartesian_diff", &CartesianTransformationServer::move_cb, this);
    }

private:
    bool move_cb(jaco2_msgs::CartesianTransformation::Request& req, jaco2_msgs::CartesianTransformation::Response & res)
    {
        robot_state::RobotState start_state(*group_.getCurrentState());
//        geometry_msgs::Pose start_pose2;
//        for(auto name : group.getJointNames()){
//            ROS_INFO_STREAM(name <<": "<< start_state.getVariablePosition(name));
//        }
        moveit_msgs::GetPositionFKRequest fk_request;
        moveit_msgs::GetPositionFKResponse fk_response;
        fk_request.header.frame_id = group_.getPlanningFrame();
        fk_request.fk_link_names.resize(1,group_.getEndEffectorLink());
        fk_request.robot_state.joint_state.name = group_.getActiveJoints();

        fk_client_.call(fk_request, fk_response);

        ROS_INFO_STREAM(fk_response);

        geometry_msgs::PoseStamped current_pose = fk_response.pose_stamped.front();

        std::vector<geometry_msgs::Pose> waypoints;

        tf::Pose trans,c_pose;
        tf::poseMsgToTF(req.request,trans);
        tf::poseMsgToTF(current_pose.pose,c_pose);

        tf::Pose target = c_pose * trans;
        geometry_msgs::Pose target_pose;
        tf::poseTFToMsg(target, target_pose);

        waypoints.push_back(target_pose);  // up and out

        ros::Time start = ros::Time::now();

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = group_.computeCartesianPath(waypoints,
                                                     0.01,  // eef_step
                                                     0.0,   // jump_threshold
                                                     trajectory);

        ros::Time end = ros::Time::now();
        ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
                 fraction * 100.0);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(15.0);
        my_plan_.trajectory_ = trajectory;
        my_plan_.planning_time_ = (end-start).toSec();
        moveit::core::robotStateToRobotStateMsg(start_state,my_plan_.start_state_);
//        my_plan_.start_state_ = start_state;
        group_.execute(my_plan_);

    }

private:

    moveit::planning_interface::MoveGroup::Plan my_plan_;
    moveit::planning_interface::MoveGroup group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    ros::ServiceClient fk_client_;
    ros::ServiceServer move_cart_server_;
};


int main(int argc, char *argv[])
{
    ros::init (argc, argv, "cartesian");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration sleep_time(1.0);
    sleep_time.sleep();
    sleep_time.sleep();




    sleep_time.sleep();








    return 0;
}
