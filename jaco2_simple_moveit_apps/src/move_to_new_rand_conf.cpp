#include <jaco2_utils/configuration_list.h>
#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_srvs/Trigger.h>

#include <signal.h>

#if ROS_VERSION_MINIMUM(1,12,0)
    #include <moveit/move_group_interface/move_group_interface.h>
    typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;
#else
    #include <moveit/move_group_interface/move_group.h>
    typedef moveit::planning_interface::MoveGroup MoveGroupInterface;
#endif

class NewRandomConfPlanner{
public:
    NewRandomConfPlanner(std::string group, std::string description, std::string planner) :
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

    void push_back_last_conf()
    {
        configurations_.configurations.push_back(last_conf_);
    }

    void save(std::string file)
    {
        configurations_.save(file);
    }

    void setThresolds(std::vector<double>& threshold)
    {
        configurations_.offsets = threshold;
    }

    jaco2_utils::Configuration& getLastConfiguration()
    {
        return configurations_.configurations.back();
    }

    bool move2NewConf()
    {
        bool collision = false;
        bool succeded = false;
        bool contains = true;
        int iteration = 0;


        while(!succeded){
            jaco2_utils::Configuration rand;
            //            std::vector<double> jvalues(configurations_.n_joints_);
            //            rand.angles = group_.getRandomJointValues();
            rand.angles.resize(configurations_.n_joints_);

            //            planningMonitor_->updatesScene();
            robot_state::RobotState& random_state = planningMonitor_->getPlanningScene()->getCurrentStateNonConst();

            random_state.setToRandomPositions();
            for(std::size_t i = 0; i < configurations_.n_joints_; ++i){
                //                random_state.setJointPositions(jointNames_[i], &(jvalues[i]));
                rand.angles[i] = *random_state.getJointPositions(jointNames_[i]);
            }

            random_state.update();
            ROS_INFO_STREAM("angles: " << rand.to_string());
            collision = checkCollision(planningMonitor_,random_state);
            ROS_INFO_STREAM("Iteration " << iteration << ". Random state is "
                            << (collision ? "in" : "not in")
                            << " collision" );
            contains = configurations_.contains(rand);

            if(!collision && !contains) {


                MoveGroupInterface::Plan my_plan;
                my_plan.trajectory_.joint_trajectory;

                group_.setJointValueTarget(rand.angles);
                group_.setStartStateToCurrentState();
                group_.setPlanningTime(3.0);
                moveit_msgs::MoveItErrorCodes success = group_.plan(my_plan);

                //                std::cout << "Test 4: Planning successfull:  "
                //                          << "error code: " << success.val << std::endl;
                if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                    success = group_.execute(my_plan);
                    //                    success = group_.move();
                    succeded = true;
                    std::cout << success << std::endl;
                    last_conf_ = rand;
                    //                    configurations_.configurations.push_back(rand);
                }
                if(success.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                    succeded = false;
                }
            }
            ++iteration;

        }
    }

    bool checkCollision(const planning_scene_monitor::PlanningSceneMonitorPtr& plm, const robot_state::RobotState& rstate )
    {

        planning_scene_monitor::LockedPlanningSceneRW ps(plm);
        ps->getCurrentStateNonConst().update();
        planning_scene::PlanningScenePtr scene = ps->diff();
        scene->decoupleParent();


        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

        collision_request.group_name = group_.getName();
        collision_result.clear();
        scene->setCurrentState(rstate);
        scene->checkCollision(collision_request, collision_result, rstate, acm);
        return collision_result.collision;
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

//        /* A pose for the box (specified relative to frame_id) */
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

//        primitive.type = primitive.CYLINDER;
//        primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.05;
//        primitive.dimensions[primitive.CYLINDER_HEIGHT]  = 1.0;
//        geometry_msgs::Pose c_pose;
//        c_pose.orientation.w = 1.0;
//        c_pose.position.x = 0.3;
//        c_pose.position.y = 0.2;
//        c_pose.position.z = 0.5;
//        collision_object.id = "cylinder";
//        collision_object.primitives.push_back(primitive);
//        collision_object.primitive_poses.push_back(c_pose);
//        collision_object.operation = collision_object.ADD;

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
    MoveGroupInterface group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;

    jaco2_utils::ConfigurationList configurations_;
};

std::shared_ptr<NewRandomConfPlanner> planner;
std::string conf_list = "/tmp/used_configurations.conf";
std::string save_file = "/tmp/used_configurations.conf";

bool newConfCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse & res)
{
    planner->move2NewConf();

    res.success = true;
    if(planner->getConfigurationSize() > 0)
        res.message = planner->last_conf_.to_string();
    return true;
}

bool saveCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse & res)
{
    planner->push_back_last_conf();
    planner->save(save_file);
    res.success = true;
    res.message = "saved file as " + save_file;
    return true;
}

void mySigintHandler(int sig)
{
    planner->save(save_file);
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "jaco_2_random_sampling");
    ros::NodeHandle node_handle("~");

    node_handle.param("configuration_list", conf_list, conf_list);
    node_handle.param("save_path", save_file, save_file);

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


    planner = std::shared_ptr<NewRandomConfPlanner>(new NewRandomConfPlanner("manipulator","robot_description","RRTkConfigDefault"));
    if(conf_list != ""){
        planner->loadConfigurations(conf_list);
    }
    planner->setThresolds(thresholds);

    if(use_box){
        planner->saftyBox(z,xm,xp,ym,yp);
    }


    ros::ServiceServer new_conf_service = node_handle.advertiseService("new_configuration", &newConfCb);
    ros::ServiceServer record_calib_service = node_handle.advertiseService("save_configurations", &saveCb);


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
