#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

//FollowJointTrajectory action server
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include <jaco2_msgs/Jaco2JointState.h>
#include <jaco2_msgs/Start.h>
#include <jaco2_msgs/Stop.h>


//TODO: ROS parameters (currently factory default)
const std::vector<double> K_P (6, 2);
const std::vector<double> K_I (6, 0);
const std::vector<double> K_D (6, 0.05);


class teleopJacoDS4 {
public:
    teleopJacoDS4(std::string group_name, ros::NodeHandle& nh);

    bool move_cart(double x, double y, double z, double roll, double pitch, double yaw);
    void publish_states(std::vector<double> vel);
    void move_torque_teaching(bool button_pressed);
    moveit_msgs::MoveItErrorCodes move_to_trajectory_start();
    void play_trajectory();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void jointStateJacoCallback(const jaco2_msgs::Jaco2JointState::ConstPtr& state);

    ros::Subscriber joy_sub_;
    ros::Subscriber joint_state_sub_;
    ros::ServiceClient fk_client_;
    ros::ServiceClient ik_client_;
    ros::ServiceClient gravity_client_;
    ros::ServiceClient emergency_stop_client_;
    ros::ServiceClient emergency_start_client_;
    ros::ServiceClient admittance_mode_client_;
    ros::ServiceClient random_configuration_sampling_client_;
    ros::ServiceClient random_configuration_saving_client_;
    ros::Publisher joint_state_pub;

    //action server
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> goal_action_client_;
    trajectory_msgs::JointTrajectory goal_joint_trajectory;            //add points (ie states from statecallback) to trajectory to follow, slightly overfitted since it interpolates

    moveit::planning_interface::MoveGroupInterface group_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    ros::Time prev_time;
    double prev_error;
    double sum_error;
    std::vector<std::pair<double,double>> joint_bounds;

    sensor_msgs::JointState current_state;
    sensor_msgs::JointState first_state;
    bool emergency_stop;

    //fake controller
    std::vector<double> position_;
    std::vector<double> last_position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<std::string> joint_names_;
    ros::Time now_;
    ros::Time last_;
    srdf::Model::Group manipulatorGroup_;
    srdf::Model::Group gripperGroup_;

    //teaching torque control
    std::vector< sensor_msgs::JointState > saved_trajectory;
    bool isRecording;
    bool doneRecording;
    bool useTeaching;

    bool randomConfigurationMode;
    std::vector<double> random_sampled_configuration_;

};

teleopJacoDS4::teleopJacoDS4(std::string group_name, ros::NodeHandle& nh_)
    : group_(group_name), goal_action_client_("/jaco_21_driver/follow_joint_trajectory/manipulator",true)

{
    ROS_INFO_STREAM("Waiting for Action Server.");
    goal_action_client_.waitForServer();
    ROS_INFO_STREAM("Action Server Found.");

    isRecording= false;
    doneRecording = false;
    prev_error = 0;
    sum_error = 0;
    prev_time = ros::Time::now();   //not Walltime since elapsed time is in reference to simulation (slower than walltime)
    emergency_stop = false;
    useTeaching = false;
    randomConfigurationMode = false;

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &teleopJacoDS4::joyCallback, this);
//    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/jaco_21_driver/out/joint_states", 10, &teleopJacoDS4::jointStateCallback, this);
    joint_state_sub_ = nh_.subscribe<jaco2_msgs::Jaco2JointState>("/jaco_21_driver/out/joint_state_acc", 10, &teleopJacoDS4::jointStateJacoCallback, this); //use this for accelerations

    fk_client_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    ros::service::waitForService("/compute_fk");
    ROS_INFO("FK Service ready");
    ik_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    ros::service::waitForService("/compute_ik");
    ROS_INFO("IK Service ready");

    gravity_client_ = nh_.serviceClient<std_srvs::SetBool>("/jaco_21_driver/in/enable_gravity_compensation_mode");
    emergency_start_client_ = nh_.serviceClient<jaco2_msgs::Start>("/jaco_21_driver/in/start");
    emergency_stop_client_ = nh_.serviceClient<jaco2_msgs::Stop>("/jaco_21_driver/in/stop");
    random_configuration_sampling_client_ = nh_.serviceClient<std_srvs::Trigger>("/jaco_2_random_sampling/new_configuration");
    random_configuration_saving_client_ = nh_.serviceClient<std_srvs::Trigger>("/jaco_2_random_sampling/save_configurations");

    jaco2_msgs::StartRequest req;
    jaco2_msgs::StartResponse res;
    emergency_start_client_.call(req,res);  //make sure at start emergency stop is disabled

    std_srvs::SetBoolRequest req_g;
    std_srvs::SetBoolResponse res_g;
    req_g.data = false;                   //at start no gravity compensation mode
    gravity_client_.call(req_g, res_g); //TODO catch error

    admittance_mode_client_ = nh_.serviceClient<std_srvs::SetBool>("/jaco_21_driver/in/enable_admittance_mode");
    std_srvs::SetBoolRequest req_a;
    std_srvs::SetBoolResponse res_a;
    req_a.data = true;                   //at start (PID) admittance mode enabled
    admittance_mode_client_.call(req_a, res_a); //TODO catch error
    ROS_INFO("Admittance control active.");

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

    if(rdf_loader.getSRDF()->getGroups().size() > 1) {
        gripperGroup_ = rdf_loader.getSRDF()->getGroups()[1];
    }

    int nJoints = manipulatorGroup_.joints_.size() + gripperGroup_.joints_.size();
    position_.resize(nJoints,0);
    last_position_.resize(nJoints,0);
    velocity_.resize(nJoints, 0);
    effort_.resize(nJoints, 0);
    joint_names_.resize(nJoints);

    last_= ros::Time::now();

    for(std::size_t i = 0; i < manipulatorGroup_.joints_.size(); ++i) {
        joint_names_[i] = manipulatorGroup_.joints_[i];
    }
    for(std::size_t i = 0; i < gripperGroup_.joints_.size(); ++i) {
        joint_names_[i + manipulatorGroup_.joints_.size()] = gripperGroup_.joints_[i];
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


    double factor = 50;        //scaling factor ie max movement on joystick = 1 cm robot movement
    double factor_roll = 2;
    double y = left_trigger_L1 ? 0 : left_analog_up_down/factor;
    double x = - left_analog_left_right/factor;
    double z = left_trigger_L1 ? - left_analog_up_down/factor: 0; //only if L1 pressed up down left analog = z axis up down, else no change
    double yaw = right_trigger_R1 ? 0 : - factor_roll*right_analog_left_right;
    double roll = right_analog_up_down;
    double pitch = right_trigger_R1 ? -right_analog_left_right : 0; //R1 + right analog left/right = yaw

    if (right_x && !emergency_stop && !randomConfigurationMode) { //move home
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

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
    if (right_o) { //emergency stop
        if (emergency_stop) {//has been stopped, start it up
            emergency_stop = false;
            ROS_INFO_STREAM("Removing emergency stop");
            jaco2_msgs::StartRequest req;
            jaco2_msgs::StartResponse res;
            emergency_start_client_.call(req,res);

        } else { //perform emergency stop
            emergency_stop = true;
            ROS_INFO_STREAM("Performing emergency stop.");
            jaco2_msgs::StopRequest req;
            jaco2_msgs::StopResponse res;
            emergency_stop_client_.call(req,res);
        }
        sleep(1);//at least one second between button presses
        return;
    }
    ros::Rate sr(2);
    if (current_state.position.size() > 1  && !useTeaching && !emergency_stop && !randomConfigurationMode){ //only if jointstatecallback has been called
        teleopJacoDS4::move_cart(x, y, z, roll, pitch, yaw);
        if (right_triangle) {
            useTeaching = true;
            ROS_INFO_STREAM("Using torque control");
            ROS_INFO_STREAM("Setting gravity compensation mode...");
            ROS_INFO_STREAM("Press square to start/play recording ");
            std_srvs::SetBoolRequest req;
            std_srvs::SetBoolResponse res;
            req.data = true;
            gravity_client_.call(req, res); //TODO catch error
//            sleep(1);
            sr.sleep();
        }
    } else  if (useTeaching && !emergency_stop && !randomConfigurationMode) {
        if (right_triangle) {
            useTeaching = false;
            ROS_INFO_STREAM("Using joystick control");
            ROS_INFO_STREAM("Turning off gravity compensation mode...");
            std_srvs::SetBoolRequest req;
            std_srvs::SetBoolResponse res;
            req.data = false;
            gravity_client_.call(req, res); //TODO catch error

            std_srvs::SetBoolRequest req_a;
            std_srvs::SetBoolResponse res_a;
            req_a.data = true;                   //at start (PID) admittance mode enabled
            admittance_mode_client_.call(req_a, res_a); //TODO catch error
            ROS_INFO_STREAM("Admittance control active.");

//            sleep(1);
            sr.sleep();
            doneRecording = false;
            isRecording = false;
            saved_trajectory.clear(); //deletes traj.
            goal_joint_trajectory.points.clear();
            ROS_INFO_STREAM("trajectory size " << saved_trajectory.size());
        }
        teleopJacoDS4::move_torque_teaching(right_square);        //(un)comment to enable/disable teaching input
    }
    //move to random configuration mode
    if (options_button) {
        if (randomConfigurationMode) {
            ROS_INFO_STREAM("Turning off random configuration sampling mode.");
            randomConfigurationMode = false;

            std_srvs::SetBoolRequest req_a;
            std_srvs::SetBoolResponse res_a;
            req_a.data = true;                   //at start admittance mode enabled
            admittance_mode_client_.call(req_a, res_a); //TODO catch error
            ROS_INFO_STREAM("Admittance control active.");
            sr.sleep();
            doneRecording = false;
            isRecording = false;
            saved_trajectory.clear(); //deletes traj.
            goal_joint_trajectory.points.clear();
        } else {
            ROS_INFO_STREAM("Turning on random configuration sampling mode.");
            randomConfigurationMode = true;
            std_srvs::SetBoolRequest req_a;
            std_srvs::SetBoolResponse res_a;
            req_a.data = true;                   //at start admittance mode enabled
            admittance_mode_client_.call(req_a, res_a); //TODO catch error
            ROS_INFO_STREAM("Admittance control active.");
             sr.sleep();
        }
    }
    if (randomConfigurationMode) {
        if (right_triangle) {
            if (doneRecording) {    //reset
                doneRecording = false;
                isRecording = false;
                saved_trajectory.clear(); //deletes traj.
                goal_joint_trajectory.points.clear();
            }
            //move to and save random configuration
            ROS_INFO_STREAM("Moving to random configuration.");
            std_srvs::TriggerRequest req_r;
            std_srvs::TriggerResponse res_r;

            isRecording= true;
            group_.rememberJointValues("startState");

            random_configuration_sampling_client_.call(req_r, res_r); //TODO catch error
            if (res_r.success) {
                ROS_INFO_STREAM("...reached configuration.");

                std_srvs::TriggerRequest req_s;
                std_srvs::TriggerResponse res_s;
                random_configuration_saving_client_.call(req_s, res_s); //saves configuration (prevents same conf. being used multiple times

                isRecording = false;
                doneRecording = true;
                sr.sleep();
                ros::Duration dur(1/2);
                dur.sleep();
                move_to_trajectory_start();
                ROS_INFO_STREAM("Moved to start.");
            } else {    //probably "rosrun jaco2_simple_moveit_apps move_to_new_rand_configuration " was not called
                ROS_INFO_STREAM("Failed to move to random configuration.");
                ROS_INFO_STREAM("Check that move_to_new_rand_configuration server is running.");
                doneRecording = false;
                isRecording = false;
                saved_trajectory.clear(); //deletes traj.
                goal_joint_trajectory.points.clear();
            }

            std::string s = res_r.message;  //get configuration, parse it and save it
            std::string delimiter = ";";
            size_t pos = 0;
            std::string token;
            std::vector<double> configuration;
            while ((pos = s.find(delimiter)) != std::string::npos) {
                token = s.substr(0, pos);
                //            ROS_INFO_STREAM("token: " << token);
                configuration.push_back(std::stod(token));
                s.erase(0, pos + delimiter.length());
            }
            random_sampled_configuration_ = configuration;
            for (int i = 0; i < configuration.size(); i++) {
//                ROS_INFO_STREAM("conf: "<< random_sampled_configuration_[i]);
            }
            sr.sleep();
        }
        if (right_square && !doneRecording) {
            ROS_INFO_STREAM("No trajectory saved!");
            sr.sleep();
        } else if (right_square && doneRecording && !isRecording) {
            play_trajectory();
        }

    }

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


void teleopJacoDS4::jointStateJacoCallback(const jaco2_msgs::Jaco2JointState::ConstPtr& state) {
    if (first_state.position.size() == 0) {
        first_state.position.clear();       //always only save first 6 joints, not fingers
        first_state.velocity.clear();
        first_state.effort.clear();
        first_state.position.insert(first_state.position.end(), state->position.begin(),state->position.begin() + 6) ;
        first_state.velocity.insert(first_state.velocity.end(), state->velocity.begin(),state->velocity.begin() + 6);
        first_state.effort.insert(first_state.effort.end(), state->effort.begin(),state->effort.begin() + 6);
    }

    current_state.position.clear();
    current_state.velocity.clear();
    current_state.effort.clear();
    current_state.position.insert(current_state.position.end(), state->position.begin(),state->position.begin() + 6) ;
    current_state.velocity.insert(current_state.velocity.end(), state->velocity.begin(),state->velocity.begin() + 6);
    current_state.effort.insert(current_state.effort.end(), state->effort.begin(),state->effort.begin() + 6);

    current_state.header = state->header;
    current_state.name = state->name;

    if (isRecording) {
        saved_trajectory.push_back(current_state);
        trajectory_msgs::JointTrajectoryPoint current_point;
        current_point.positions.insert(current_point.positions.end(), state->position.begin(),state->position.begin() + 6) ;
        current_point.velocities.insert(current_point.velocities.end(), state->velocity.begin(),state->velocity.begin() + 6);
        current_point.effort.insert(current_point.effort.end(), state->effort.begin(),state->effort.begin() + 6);
        current_point.accelerations.insert(current_point.accelerations.end(), state->acceleration.begin(),state->acceleration.begin() + 6);
        current_point.time_from_start = ros::Time::now() - saved_trajectory[0].header.stamp;
        goal_joint_trajectory.points.push_back(current_point);
    }

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

moveit_msgs::MoveItErrorCodes teleopJacoDS4::move_to_trajectory_start(){
    if (saved_trajectory[0].position != current_state.position) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        group_.setPlannerId("RRTConnectkConfigDefault");
        group_.setStartStateToCurrentState();
        group_.setPlanningTime(1.0);
        group_.setNamedTarget("startState");

        moveit_msgs::MoveItErrorCodes success = group_.plan(my_plan);
        if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Success! Moved to start");
//        group.execute(my_plan);
            success = group_.move();
        }
        return success;
    }
}

void teleopJacoDS4::move_torque_teaching(bool button_pressed) { //button press starts, stops and plays recording
    //call service torque control
    ros::Rate sr(2); //sleep rate
    //wait for record, record velocity, wait for end record
    if (!isRecording && !doneRecording && button_pressed) {
        sr.sleep();
        std::cout << "Starting recording... "<< std::endl;
        isRecording= true;
        group_.rememberJointValues("startState");
        return;
    } else if (isRecording && !doneRecording && button_pressed) {
        std::cout<<"... recording finished."<<std::endl;
        isRecording = false;
        doneRecording = true;
        sr.sleep();
        ros::Duration dur(1/2);
        dur.sleep();
        move_to_trajectory_start();
        return;
    }

    //on button press play recorded data (moves back to start of trajectory first)

    else if (doneRecording && button_pressed) {
        play_trajectory();
    }
}

//enters addmittance mode, plays back saved trajectory and moves back to trajectory start
void teleopJacoDS4::play_trajectory() {
    std_srvs::SetBoolRequest req_a;
    std_srvs::SetBoolResponse res_a;
    req_a.data = true;                   //at start (PID) admittance mode enabled
    admittance_mode_client_.call(req_a, res_a); //TODO catch error
    ROS_INFO_STREAM("Admittance control active.");
    std::cout << "Starting playback of recording..."<<std::endl;

//        std_srvs::SetBoolRequest req;
//        std_srvs::SetBoolResponse res;
//        req.data = true;
//        gravity_client_.call(req, res); //TODO catch error

    moveit_msgs::MoveItErrorCodes moved_to_start = move_to_trajectory_start();
    if (moved_to_start.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO_STREAM("Could not plan to start pose.");   //do not execute trajectory if not at start pose
        return;
    }

    ROS_INFO_STREAM("Size of saved trajectory "<< saved_trajectory.size());

    //playback of trajectory using velocity @60hz
//        ros::Rate r(60);
//        for (int i = 0; i < saved_trajectory.size(); i++) {
//            teleopJacoDS4::publish_states(saved_trajectory[i].velocity);
//            r.sleep();
//        }
//        ROS_INFO_STREAM("Playback Finished.");

    //playback using Action Server
    if (goal_action_client_.isServerConnected()) {
        ROS_INFO_STREAM("Starting Playback...");
        control_msgs::FollowJointTrajectoryGoal goal_action_msg;
        goal_joint_trajectory.header.stamp = saved_trajectory[0].header.stamp;          //time_from_start is relative to this timestamp (should be only one needed)
        goal_joint_trajectory.joint_names.clear();  //prevent adding of multiple sets of joint names
        goal_joint_trajectory.joint_names.insert(goal_joint_trajectory.joint_names.end(), saved_trajectory[0].name.begin(),saved_trajectory[0].name.begin() + 6);            //set joint names

        goal_action_msg.trajectory = goal_joint_trajectory;

        goal_action_client_.sendGoal(goal_action_msg);
        if ((goal_action_client_.getResult())->error_code == 0) {
            ros::Duration traj_dur = goal_joint_trajectory.points.back().time_from_start;
            traj_dur.sleep();
            ros::Duration dur(1); //wait extra 1 second after execution before moving back
            dur.sleep();
            ROS_INFO_STREAM("Playback Finished. Moving to start.");
            move_to_trajectory_start();
        }

    } else {
        ROS_INFO_STREAM("Action Server not connected, cannot execute trajectory.");
    }
}



int main(int argc, char *argv[])
{
    ros::init (argc, argv, "ds4_PID_teleop");
    ros::NodeHandle nh_("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    teleopJacoDS4 teleop_jaco("manipulator", nh_);
    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
