#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

#if ROS_VERSION_MINIMUM(1,12,0)
    #include <moveit/move_group_interface/move_group_interface.h>
    typedef moveit::planning_interface::MoveGroupInterface MoveGroup;
#else
    #include <moveit/move_group_interface/move_group.h>
    typedef moveit::planning_interface::MoveGroup MoveGroup;
#endif

int main(int argc, char *argv[])
{
    if(argc > 6) {
        std::vector<double> config = {std::atof(argv[1]),
                                      std::atof(argv[2]),
                                      std::atof(argv[3]),
                                      std::atof(argv[4]),
                                      std::atof(argv[5]),
                                      std::atof(argv[6])};

        ros::init (argc, argv, "move_to_config");
        ros::NodeHandle node_handle("~");
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::Duration sleep_time(1.0);
        sleep_time.sleep();
        sleep_time.sleep();

        MoveGroup::Plan my_plan;
        MoveGroup group("manipulator");
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


        group.setPlannerId("RRTkConfigDefault");
        group.setStartStateToCurrentState();
        group.setPlanningTime(5.0);
        group.setJointValueTarget(config);


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
    else
    {
        return 42;
    }
}

