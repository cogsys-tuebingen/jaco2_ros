#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#if ROS_VERSION_MINIMUM(1, 12, 0)
    #include <moveit/move_group_interface/move_group_interface.h>
#else
    #include <moveit/move_group_interface/move_group.h>
#endif
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>


int main(int argc, char** argv) {
    ros::init (argc, argv, "distFunction");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //    sleep(4.0);


#if ROS_VERSION_MINIMUM(1, 12, 0)
    moveit::planning_interface::MoveGroupInterface group_("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;
    planningMonitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
#else
    moveit::planning_interface::MoveGroup group_("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;
    planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
#endif
    std::vector<std::string> jointNames_;
    jointNames_ =  planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();

    group_.setStartStateToCurrentState();

    //###############################################################################################################################
    //###############################################################################################################################
    //    saftyBox();
    double z = -0.2; //formerly 0
    double xm = -1;
    double xp = 1;
    double ym = -1;
    double yp = 1;


    std::vector<moveit_msgs::CollisionObject> collision_objects;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.5;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  1 + 0.5* primitive.dimensions[0];
    box_pose.position.y =  1 + 0.5* primitive.dimensions[1];
    box_pose.position.z =  0 + 0.5* primitive.dimensions[2];
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "dice";
    collision_object.header.frame_id = group_.getPlanningFrame();

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;



    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");

    collision_objects.push_back(collision_object);
    planning_scene_interface_.addCollisionObjects(collision_objects);
    //###############################################################################################################################
    //###############################################################################################################################

    //move_new_rand_conf_call
    planning_scene_monitor::LockedPlanningSceneRW ps(planningMonitor_);
    //    robot_state::RobotState state = ps->getCurrentStateNonConst();
    ps->getCurrentStateNonConst().update();
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();

    ROS_INFO_STREAM("Active Collision Detector: " << scene->getActiveCollisionDetectorName());

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

    collision_request.distance = true;
    collision_request.group_name = group_.getName();
    collision_result.clear();

    robot_state::RobotState state = scene->getCurrentStateNonConst();
    //    robot_state::RobotState& state = planningMonitor_->getPlanningScene()->getCurrentStateNonConst();
    state.update();
    state.updateCollisionBodyTransforms();
    std::vector<double> q = {0,M_PI,M_PI,0,0,0};
    std::vector<std::string> names = planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();

    for(int i = 0; i < 2; ++i)
    {
        if(i==1){
            //            state.setToRandomPositions(); // works as well
            auto it_q = q.begin();
            for(auto lname : names){
                state.setVariablePosition(lname, *it_q);
                ++it_q;
            }
            //            state.update();
        }

        for(auto lname : names){
            ROS_INFO_STREAM(lname << " position " << *state.getJointPositions(lname));
        }
        scene->setCurrentState(state);


        std::map<std::string, moveit_msgs::CollisionObject> c_objects_map = planning_scene_interface_.getObjects();
        for(auto& kv : c_objects_map){
            //            ROS_INFO_STREAM( kv.first << " has value " << kv.second );
            scene->processCollisionObjectMsg(kv.second);
        }

        //    for(auto i : collision_objects)
        //        scene->processCollisionObjectMsg(i);




        double distance_distanceToCollision = scene->distanceToCollision(state);
        ROS_INFO_STREAM("distanceToCollision(current conf "<< i << "): " << distance_distanceToCollision);
    }

    double distance_distanceRobot =  scene->getCollisionWorld()->distanceRobot(*(scene->getCollisionRobot()), state, scene->getAllowedCollisionMatrix());
    ROS_INFO_STREAM("calling distancerobot (current conf) : "<< distance_distanceRobot);


    //end new_rand_conf_call



    //    planning_scene::PlanningScenePtr planning_scene_ptr(new planning_scene::PlanningScene( scene_->getRobotModel()));
    //    boost::shared_ptr<collision_detection::CollisionDetectorAllocator> fcl_cd(
    //                collision_detection::CollisionDetectorAllocatorFCL::create());
    //    planning_scene_ptr->setActiveCollisionDetector(fcl_cd, true);



    return 0;

}
