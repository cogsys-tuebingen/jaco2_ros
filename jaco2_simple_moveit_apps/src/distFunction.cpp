#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>


int main(int argc, char** argv) {
    ros::init (argc, argv, "distFunction");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
//    sleep(4.0);

    moveit::planning_interface::MoveGroup group_("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;
    planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
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
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 0.01;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.0;
    box_pose.position.y =  0.0;
    box_pose.position.z =  z;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group_.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.id = "ground_plan";

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;



    collision_objects.push_back(collision_object);

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.01;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 2.0;

    box_pose.orientation.w = 1.0;
    box_pose.position.x =  xp;
    box_pose.position.y =  0.0;
    box_pose.position.z =  1.0;

    collision_object.id = "wall_1";

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.01;
    primitive.dimensions[1] = 2.0;

    box_pose.orientation.w = 1.0;
    box_pose.position.x =  xm;
    box_pose.position.y =  0.0;


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 0.01;

    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.0;
    box_pose.position.y =  yp;

    collision_object.id = "wall_3";

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2.0;
    primitive.dimensions[1] = 0.01;

    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.0;
    box_pose.position.y =  ym;

    collision_object.id = "wall_4";

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

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

    scene->setCurrentState(state);
    scene->checkCollision(collision_request, collision_result, state, acm);
    ROS_ERROR_STREAM("result.distance (new rand conf) " << collision_result.distance);



    for(auto i : collision_objects)
        scene->processCollisionObjectMsg(i);




    double distance_distanceToCollision = scene->distanceToCollision(state);
    ROS_ERROR_STREAM("distanceToCollision(randconf): " << distance_distanceToCollision);

    double distance_distanceRobot =  scene->getCollisionWorld()->distanceRobot(*(scene->getCollisionRobot()), state, scene->getAllowedCollisionMatrix());
    ROS_ERROR_STREAM("calling distancerobot (randconf) : "<< distance_distanceRobot);


    //end new_rand_conf_call



    //    planning_scene::PlanningScenePtr planning_scene_ptr(new planning_scene::PlanningScene( scene_->getRobotModel()));
    //    boost::shared_ptr<collision_detection::CollisionDetectorAllocator> fcl_cd(
    //                collision_detection::CollisionDetectorAllocatorFCL::create());
    //    planning_scene_ptr->setActiveCollisionDetector(fcl_cd, true);



    return 0;

}
