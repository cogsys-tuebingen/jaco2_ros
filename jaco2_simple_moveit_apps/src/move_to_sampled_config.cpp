#include <jaco2_utils/configuration_list.h>
#include <jaco2_msgs/SetSampledConfigService.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_srvs/Trigger.h>

#include <signal.h>

class MoveToSampledConfPlanner{
public:
    MoveToSampledConfPlanner(std::string group, std::string description, std::string planner) :
        group_(group)
    {
#if ROS_VERSION_MINIMUM(1, 12, 0)
        planningMonitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
#else
        planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
#endif
        jointNames_ =  planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();

        group_.setPlannerId(planner);
        group_.setStartStateToCurrentState();
        group_.setPlanningTime(5.0);

        configurations_.n_joints_ = jointNames_.size();
        configurations_.jointNames = jointNames_;

        for(std::size_t i = 0; i < jointNames_.size(); ++i)
        {
            configurations_.offsets.push_back(0.05);
        }

    }

    bool loadConfigurations(std::string file)
    {
        bool res;
        bool loaded = configurations_.load(file);
        if(loaded){
            return configurations_.jointNames.size() == jointNames_.size();
        }
        else{

            configurations_.n_joints_ = jointNames_.size();
            configurations_.jointNames = jointNames_;
        }
        return loaded;
    }

    bool move2NewConf(const std::size_t id, int result)
    {

        if(id < configurations_.configurations.size()){
            group_.setJointValueTarget(configurations_.configurations[id].angles);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit_msgs::MoveItErrorCodes success = group_.plan(my_plan);
            if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_INFO("Success! Now move");
                //        group.execute(my_plan);
                success = group_.move();
            }
            result = success.val;
            return (success.val == moveit_msgs::MoveItErrorCodes::SUCCESS);
        }
        else
        {
            result = jaco2_msgs::SetSampledConfigServiceResponse::INVALID_INPUT;
            return false;
        }

    }



    void saftyBox(double z = 0, double xm = -1, double xp = 1,
                  double ym = -1, double yp = 1)
    {
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
        collision_object.id = "ground_plane";

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
        //        bool suc =  group_.attachObject("ground_plan","root");
        //        std::cout << suc << std::endl;
    }

    std::size_t getConfigurationSize()
    {
        return configurations_.configurations.size();
    }

public:
    jaco2_utils::Configuration last_conf_;
private:

    std::vector<std::string> jointNames_;
    moveit::planning_interface::MoveGroupInterface group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;

    jaco2_utils::ConfigurationList configurations_;
};

std::shared_ptr<MoveToSampledConfPlanner> planner;
std::string conf_list = "/tmp/used_configurations.conf";

bool gotoConfCb(jaco2_msgs::SetSampledConfigServiceRequest &req, jaco2_msgs::SetSampledConfigServiceResponse & res)
{

    return planner->move2NewConf(req.config,res.result);
}

void mySigintHandler(int sig)
{
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "jaco_2_move_to_sampling");
    ros::NodeHandle node_handle("~");

    node_handle.param("configuration_list", conf_list, conf_list);


    std::vector<double> thresholds;
    node_handle.param("thresholds",thresholds, thresholds);

    bool use_box = true;
    node_handle.param("use_savety_box",use_box, use_box);

    double z = 0;
    double xm = -1;
    double xp = 1;
    double ym = -1;
    double yp = 1;

    node_handle.param("z_coord",z, z);
    node_handle.param("xm_coord",xm, xm);
    node_handle.param("xp_coord",xp, xp);
    node_handle.param("ym_coord",ym, ym);
    node_handle.param("yp_coord",yp, yp);


    planner = std::shared_ptr<MoveToSampledConfPlanner>(new MoveToSampledConfPlanner("manipulator","robot_description","RRTkConfigDefault"));
    if(conf_list != ""){
        planner->loadConfigurations(conf_list);
    }

    if(use_box){
        planner->saftyBox(z,xm,xp,ym,yp);
    }


    ros::ServiceServer go_to_service = node_handle.advertiseService("go_to", &gotoConfCb);



    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration sleep_time(1.0);
    sleep_time.sleep();
    sleep_time.sleep();
    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        signal(SIGINT, mySigintHandler);
        r.sleep();
    }

    //    ros::shutdown();
    return 0;
}
