#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>

#include <jaco2_msgs/CartesianTransformation.h>
#include <jaco2_msgs/JointVelocity.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

#include <tf/tf.h>

//fake controller
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>


//TODO: ROS parameters (currently factory default)
const std::vector<double> K_P (6, 2);
const std::vector<double> K_I (6, 0);
const std::vector<double> K_D (6, 0.05);


class teleopJacoDS4 {
public:
    teleopJacoDS4(std::string group_name, ros::NodeHandle& nh);

    bool move_cart(double x, double y, double z, double roll, double pitch, double yaw);
    void publish_states(std::vector<double> vel);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& state);

    ros::Subscriber joy_sub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceClient fk_client_;
    ros::ServiceClient ik_client_;
    ros::Publisher joint_state_pub;
    moveit::planning_interface::MoveGroupInterface group_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    ros::Time prev_time;
    double prev_error;
    double sum_error;
    std::vector<std::pair<double,double>> joint_bounds;

    sensor_msgs::JointState current_state;
    sensor_msgs::JointState first_state;
    bool stop_em;

    //fake controller
    std::vector<double> position_;
    std::vector<double> lastPosition_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<std::string> jointNames_;
    ros::Time now_;
    ros::Time last_;
    srdf::Model::Group manipulatorGroup_;
    srdf::Model::Group gripperGroup_;

};

teleopJacoDS4::teleopJacoDS4(std::string group_name, ros::NodeHandle& nh_) : group_(group_name) {
    prev_error = 0;
    sum_error = 0;
    prev_time = ros::Time::now();   //not Walltime since elapsed time is in reference to simulation (slower than walltime)
    stop_em = false;

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &teleopJacoDS4::joyCallback, this);
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/jaco_21_driver/out/joint_states", 10, &teleopJacoDS4::jointStateCallback, this);

    fk_client_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    ros::service::waitForService("/compute_fk");
    ROS_INFO("FK Service ready");
    ik_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    ros::service::waitForService("/compute_ik");
    ROS_INFO("IK Service ready");


    //saves joint_bounds
    const moveit::core::JointModelGroup* model_group (group_.getRobotModel()->getJointModelGroup("manipulator"));
    int num_joints = model_group->getActiveJointModels().size();
    const std::vector<const moveit::core::JointModel *> joint_model(model_group->getJointModels());

    for (int i = 0; i < num_joints; i++) {
        const moveit::core::JointModel::Bounds bounds(joint_model[i]->getVariableBounds());   //get vector of variablebounds

        for (int j = 0; j < bounds.size(); j++) {       //iterate through, each varbounds has min,max pos (per joint)
            joint_bounds.push_back(std::pair<double, double>(bounds[j].min_position_, bounds[j].max_position_));
        }
    }

    //fake controller

    std::string jointStateTopic = "/jaco_21_driver/in/joint_velocity";
    std::string robot_description = "robot_description";
    joint_state_pub = nh_.advertise<jaco2_msgs::JointVelocity>(jointStateTopic, 1);
    rdf_loader::RDFLoader rdf_loader(robot_description);

    manipulatorGroup_ = rdf_loader.getSRDF()->getGroups()[0];

    if(rdf_loader.getSRDF()->getGroups().size() > 1)
    {
        gripperGroup_ = rdf_loader.getSRDF()->getGroups()[1];
    }

    int nJoints = manipulatorGroup_.joints_.size() + gripperGroup_.joints_.size();
    position_.resize(nJoints,0);
    lastPosition_.resize(nJoints,0);
    velocity_.resize(nJoints, 0);
    effort_.resize(nJoints, 0);
    jointNames_.resize(nJoints);

    last_= ros::Time::now();

    for(std::size_t i = 0; i < manipulatorGroup_.joints_.size(); ++i)
    {
        jointNames_[i] = manipulatorGroup_.joints_[i];
    }
    for(std::size_t i = 0; i < gripperGroup_.joints_.size(); ++i)
    {
        jointNames_[i + manipulatorGroup_.joints_.size()] = gripperGroup_.joints_[i];
    }



}


//tentative keymap:
//left analog up/down = forward, backward (-y axis robot frame)
//left analog left/right = left,right (x-axis)
//L1 + left analog up/down = up/down (Z-axis)
//right analog up/down = pitch
//right analog left/right= roll
//R1 + right analog left/right  = yaw

void teleopJacoDS4::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    //axes
    double left_analog_left_right = joy->axes[0]; //left = 1, right = -1
    double left_analog_up_down = joy->axes[1]; //up = 1, down = -1
    double right_analog_left_right = joy->axes[2]; //left = 1, right = -1
    double left_trigger_L2_axis = joy->axes[3]; //unpressed = 1, pressed = -1
    double right_trigger_R2_axis = joy->axes[4]; //unpressed = 1, pressed = -1
    double right_analog_up_down = joy->axes[5]; //up = 1, down = -1

    double left_arrows_left_right = joy->axes[9]; //left = 1, right = -1   (binary)
    double left_arrows_up_down = joy->axes[10]; //up= 1, down= -1   (binary)

    //buttons
    int right_square = joy->buttons[0];
    int right_x = joy->buttons[1];
    int right_o = joy->buttons[2];
    int right_triangle = joy->buttons[3];
    int left_trigger_L1 = joy->buttons[4];
    int right_trigger_R1 = joy->buttons[5];
    int left_trigger_L2_button = joy->buttons[6]; //same as axes above, with binary 01 pressed or not
    int right_trigger_R2_button = joy->buttons[7];
    int share_button = joy->buttons[8];
    int options_button = joy->buttons[9];
    int left_analog_button = joy->buttons[10];
    int right_analog_button = joy->buttons[11];
    int ps4_button = joy->buttons[12];
    int touchpad_button = joy->buttons[13]; //anywhere on touchpad is pressed


    double factor = 100;        //scaling factor ie max movement on joystick = 1 cm robot movement
    double factor_roll = 2;
    double y = left_trigger_L1 ? 0 : left_analog_up_down/factor;
    double x = - left_analog_left_right/factor;
    double z = left_trigger_L1 ? - left_analog_up_down/factor: 0; //only if L1 pressed up down left analog = z axis up down, else no change
    double yaw = right_trigger_R1 ? 0 : - factor_roll*right_analog_left_right;
    double roll = right_analog_up_down;
    double pitch = right_trigger_R1 ? -right_analog_left_right : 0; //R1 + right analog left/right = yaw

    if (right_x) { //move home
        moveit::planning_interface::MoveGroup::Plan my_plan;


        group_.setPlannerId("RRTkConfigDefault");
        group_.setStartStateToCurrentState();
        group_.setPlanningTime(5.0);
        group_.setNamedTarget("home");


        moveit_msgs::MoveItErrorCodes success = group_.plan(my_plan);
        if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Success! Moved home");
    //        group.execute(my_plan);
            success = group_.move();
        }
        return;
    }
    if (right_o) { //emergency stop (doesnt work yet)
        std::vector<double> new_vel(6,0);
        teleopJacoDS4::publish_states(new_vel);
        stop_em = stop_em ? false : true; //toggle value (ie resume)
        return;
    }


    if (current_state.position.size() > 1 && !stop_em){ //only if jointstatecallback has been called
        teleopJacoDS4::move_cart(x, y, z, roll, pitch, yaw);
    }

}

void teleopJacoDS4::jointStateCallback(const sensor_msgs::JointState::ConstPtr& state) {
    if (first_state.position.size() == 0) {

        first_state.position = state->position;
        first_state.velocity = state->velocity;
        first_state.effort = state->effort;
    }

    current_state.position = state->position;
    current_state.velocity = state->velocity;
    current_state.effort = state->effort;
}


bool teleopJacoDS4::move_cart(double x, double y, double z, double roll, double pitch, double yaw) {

    moveit_msgs::GetPositionFKRequest fk_request;
    moveit_msgs::GetPositionFKResponse fk_response;
    fk_request.header.frame_id = group_.getPlanningFrame();
    fk_request.fk_link_names.resize(1,group_.getEndEffectorLink());
    fk_request.robot_state.joint_state.name = group_.getActiveJoints();
    fk_request.robot_state.joint_state.position = current_state.position;

    fk_client_.call(fk_request, fk_response);

    geometry_msgs::PoseStamped current_pose = fk_response.pose_stamped.front();

    tf::Pose c_pose;
    tf::Pose trans;
    trans.setIdentity();
    trans.setOrigin(tf::Vector3(x, y, z));
    tf::Pose rot;
    tf::Quaternion q;
    q.setRPY(roll/180.0*M_PI, pitch/180.0*M_PI, yaw/180.0*M_PI );
    rot.setRotation(q);

    tf::poseMsgToTF(current_pose.pose,c_pose);

    tf::Pose target = trans * c_pose * rot;

    geometry_msgs::Pose result_pose;
    tf::poseTFToMsg(target, result_pose);

    //calculate IK
    moveit_msgs::GetPositionIKRequest ik_request;
    moveit_msgs::GetPositionIKResponse ik_response;
    ik_request.ik_request.group_name = "manipulator";
    ik_request.ik_request.pose_stamped.pose = result_pose;
    ik_request.ik_request.avoid_collisions = true;

    ik_client_.call(ik_request, ik_response);
    if (ik_response.error_code.val != 1) { //only continue if ik was successful
        return false;
    }

    std::vector<double> curr_q(6);
    std::vector<double> goal_q(6);
    for(int i = 0; i < curr_q.size(); i++) {
        curr_q[i] = current_state.position[i];
        goal_q[i] = ik_response.solution.joint_state.position[i];
        position_[i] = curr_q[i];
    }

    //TODO: more robust collision checking
    bool valid_goal = true;
    for (int i = 0; i < joint_bounds.size(); i++) {
        if (!(joint_bounds[i].first <= goal_q[i] <= joint_bounds[i].second)) {
           valid_goal = false;
        }
    }
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    scene->checkSelfCollision(collision_request, collision_result);
    valid_goal = collision_result.collision;
    std::vector<double> new_vel(goal_q.size());
    if (valid_goal) {
        //PID controller
        for (int i = 0; i < new_vel.size(); i++) {
            double e = goal_q[i] - curr_q[i];
            double dt = (ros::Time::now() - prev_time ).toSec();
            double de = prev_error - e;
            sum_error += e * dt;
            new_vel[i] = K_P[i] * e + K_I[i] * sum_error + K_D[i] * de/dt;
        }
        prev_time = ros::Time::now(); //set current time for next step
    }

    teleopJacoDS4::publish_states(new_vel);
}

void teleopJacoDS4::publish_states(std::vector<double> vel) {

    jaco2_msgs::JointVelocity vel_msg;
    vel_msg.joint1 = vel[0];
    vel_msg.joint2 = vel[1];
    vel_msg.joint3 = vel[2];
    vel_msg.joint4 = vel[3];
    vel_msg.joint5 = vel[4];
    vel_msg.joint6 = vel[5];
    joint_state_pub.publish(vel_msg);

}



int main(int argc, char *argv[])
{
    ros::init (argc, argv, "ds4_PID_teleop");
    ros::NodeHandle nh_("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Duration sleep_time (1.0);
    sleep_time.sleep();
    sleep_time.sleep();

    teleopJacoDS4 teleop_jaco("manipulator", nh_);
    ros::Rate r(20);
    sleep_time.sleep();
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
