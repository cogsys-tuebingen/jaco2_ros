#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "home_arm");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration sleep_time(1.0);
    sleep_time.sleep();
    sleep_time.sleep();

    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit::planning_interface::MoveGroup group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    group.setPlannerId("RRTkConfigDefault");
    group.setStartStateToCurrentState();
    group.setPlanningTime(5.0);
    group.setNamedTarget("home");


    moveit_msgs::MoveItErrorCodes success = group.plan(my_plan);
    if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Success! Now move");
//        group.execute(my_plan);
        success = group.move();
    }
    spinner.stop();
//    ros::shutdown();
    return 0;
}

