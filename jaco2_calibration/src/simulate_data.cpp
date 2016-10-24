#include <vector>
#include <random>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <jaco2_msgs/Jaco2Sensor.h>
#include <jaco2_msgs/CalibAcc.h>
#include <jaco2_msgs/JointAngles.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <jaco2_calibration/jaco2_calibration.h>
#include <jaco2_calibration_utils/dynamic_calibration_sample.hpp>
#include <jaco2_calibration_utils/acceleration_samples.hpp>
#include <jaco2_calibration_utils/jaco2_calibration_io.h>


class SimCalibNode
{
public:
    SimCalibNode(bool genData, std::string urdf_param, std::string root, std::string tip, int numberOfSamples)
        : private_nh_("~"),
          genData_(genData),
          initial_(true),
          initialSensor_(true),
          notCalib_(true),
          calibAcc_(false),
          calibration_(urdf_param,root,tip),
          numberOfSamples_(numberOfSamples),
          currentSamples_(0),
          g_counter_(0),
          moveGroup_("manipulator"),
          dynSolver_(urdf_param, root, tip),
          distribution_(0,0.4)
    {
        boost::function<void(const sensor_msgs::JointStateConstPtr&)> cb = boost::bind(&SimCalibNode::jointStateCb, this, _1);
        subJointState_ = private_nh_.subscribe("/jaco_arm_driver/out/joint_states", 10, cb);

        calibration_.setGravityMagnitude(1.0);
        calibration_.setInitAccSamples(500);
//        calibServiceServer_ = private_nh_.advertiseService("calibrate_acc", &SimCalibNode::changeCalibCallback, this);

        moveGroup_.setPlannerId("RRTkConfigDefault");
//        moveGroup_.set
//        moveGroup_.setStartStateToCurrentState();
        moveGroup_.setPlanningTime(2.0);
//        moveGroup_.setGoalPositionTolerance(0.01);
//        moveGroup_.setGoalOrientationTolerance(0.05);

        planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        jointGroupNames_ =  planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();

        //    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);



    }

    void jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
    {
        if(initial_){
            lastTime_ = ros::Time::now();
            initial_ = false;
        }
        ros::Time now = ros::Time::now();
        ros::Duration dt = now-lastTime_;
        lastTime_ = now;
        dt_ = dt.toSec();
        Jaco2Calibration::DynamicCalibrationSample sample;
        sample.time = dt_;

        double tdiff = dt_;
        if (samples_.size() > 2) {
            tdiff = samples_.at(samples_.size() -1).time + samples_.at(samples_.size() -2).time;
        }
        for(std::size_t i = 0; i < 6; ++i){
            sample.jointPos[i] = msg->position[i];
            sample.jointVel[i] = msg->velocity[i];
//            sample.jointTorque[i] = msg->effort[i];
//            sample.jointAcc[i] = msg->acceleration[i];
            if(samples_.size() > 2 && dt_ !=0)
            {
                sample.jointAcc[i] = (sample.jointVel[i] - samples_.at(samples_.size() -2).jointVel[i])/(tdiff);
            }
            else{
                sample.jointAcc[i] = 0;
            }
        }
        dynSolver_.getTorques(sample.jointPos, sample.jointVel, sample.jointAcc,sample.jointTorque);
        // add white noise
        for(std::size_t i = 0; i < 6; ++i) {
            sample.jointTorque[i] += (double) distribution_(generator_);
        }

        //estimate jaco's base acceleration
        sample.gravity = Eigen::Vector3d(0, 0, -9.81);
        samples_.push_back(sample);

        ++currentSamples_;
        //        ROS_INFO_STREAM("Recoding_Data");
    }


//    bool changeCalibCallback(jaco2_msgs::CalibAcc::Request & req, jaco2_msgs::CalibAcc::Response& res)
//    {
//        calibAcc_ = req.calib_acc;
//        if(calibAcc_){
//            res.calib_acc_result = "Starting Accelerometer Calibration.";
//        }
//        else{
//            res.calib_acc_result = "Starting Dynamic Parameter Calibration.";
//        }
//        notCalib_ = true;
//        return true;
//    }


    bool checkCollision(const planning_scene_monitor::PlanningSceneMonitorPtr& plm, const robot_state::RobotState& rstate )
    {

        planning_scene_monitor::LockedPlanningSceneRW ps(plm);
        ps->getCurrentStateNonConst().update();
        planning_scene::PlanningScenePtr scene = ps->diff();
        scene->decoupleParent();


        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();


        collision_request.group_name = "manipulator";
        collision_result.clear();
        scene->setCurrentState(rstate);
        scene->checkCollision(collision_request, collision_result, rstate, acm);
        return collision_result.collision;
    }

    void tick(bool& done)
    {
        ros::spinOnce();
        if(genData_ && currentSamples_ % 20 == 0 )
        {
            //TODO IMPLEMENT MOVING
            bool collision = true;
            while(collision)
            {
                std::vector<double> jvalues = moveGroup_.getRandomJointValues();

                robot_state::RobotState& random_state = planningMonitor_->getPlanningScene()->getCurrentStateNonConst();


                for(std::size_t i = 0; i < jointGroupNames_.size(); ++i){
                    random_state.setJointPositions(jointGroupNames_[i], &(jvalues[i]));
                }

                random_state.update();

                collision = checkCollision(planningMonitor_,random_state);
                ROS_INFO_STREAM("Random state is "
                                << (collision ? "in" : "not in")
                                << " collision" );


                if(!collision) {

                    moveit::planning_interface::MoveGroup::Plan my_plan;
                    moveGroup_.setJointValueTarget(jvalues);
                    //                    moveGroup_.setStartStateToCurrentState();
                    moveGroup_.setPlanningTime(3.0);
                    moveit_msgs::MoveItErrorCodes success = moveGroup_.plan(my_plan);

                    //                    std::cout << "Test 4: Planning successfull:  "
                    //                              << "error code: " << success.val << std::endl;
                    if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                        //                        Group.execute(my_plan);
                        success = moveGroup_.move();
                    }
                    if(success.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                        collision = true;
                    }
                }
            }

        }
        done = currentSamples_ >= numberOfSamples_;
        ROS_INFO_STREAM("recording data ... samples: " <<currentSamples_);
        //        std::cout << "recording data ... samples: " << currentSamples_  << std::endl;

    }


    void calibrate(bool calib_acc)
    {
        if(currentSamples_ > numberOfSamples_ && notCalib_)
        {
//            std::cout << "calibrating ... " << std::endl;
//            notCalib_ = false;
//            accSamples_.save("/tmp/acc_samples.txt");
//            Jaco2Calibration::save("/tmp/data.txt",samples_);
            if(calibAcc_ || calib_acc){
//                bool succ = calibration_.calibrateAcc(accSamples_);
//                if(succ){
//                    Jaco2Calibration::save("/tmp/acc_calib.txt",calibration_.getAccCalibration());
//                }
                ROS_WARN_STREAM(" Simulated Accelerometer Calibration not implemented, yet.");

            }
            else{
                Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/dyn_samples_sim.txt", samples_);
                int ec = calibration_.calibrateCoMandInertia(samples_);
                std::vector<Jaco2Calibration::DynamicCalibratedParameters> dynparams;
                if(ec > -1){
                    dynparams = calibration_.getDynamicCalibration();
                }
                Jaco2Calibration::Jaco2CalibrationIO::save("/tmp/sim_params.txt", dynparams);

            }
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
        collision_object.header.frame_id = moveGroup_.getPlanningFrame();

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
        planningSceneInterface_.addCollisionObjects(collision_objects);
    }


private:
    ros::NodeHandle private_nh_;
    bool genData_;
    bool initial_;
    bool initialSensor_;
    bool notCalib_;
    bool calibAcc_;
    Jaco2Calibration::Jaco2Calibration calibration_;
    int numberOfSamples_;
    int currentSamples_;
    int g_counter_;
    ros::Subscriber subJointState_;
    ros::Subscriber subSensors_;
    ros::Subscriber subJointAcc_;
    std::vector<Jaco2Calibration::DynamicCalibrationSample> samples_;
    std::vector<Eigen::Vector3d> gravity_;
//    Jaco2Calibration::AccelerationSamples accSamples_;
    ros::Time lastTime_;
    double dt_;
    jaco2_msgs::Jaco2Sensor jacoSensorMsg_;
    ros::ServiceServer calibServiceServer_;
    std::vector<Eigen::Vector3d> gsum_;
    std::vector<std::string> jointGroupNames_;
    moveit::planning_interface::MoveGroup moveGroup_;
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface_;
    planning_scene_monitor::PlanningSceneMonitorPtr  planningMonitor_;
    Jaco2KinematicsDynamicsModel dynSolver_;
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
    //    ros::Publisher display_publisher_;
    //    moveit_msgs::DisplayTrajectory display_trajectory_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_calibration_node");
    SimCalibNode node(true,"/robot_description","jaco_link_base","jaco_link_hand",20000);
    ros::Rate r(25);

    ros::AsyncSpinner spinner(1);
    spinner.start();

//    node.saftyBox(0, -1.5,1.5, -1.5, 1.5 );
    bool done = false;
    while (ros::ok() && !done) {
        node.tick(done);
        //        ros::spinOnce();
        r.sleep();
    }

    spinner.stop();

    node.calibrate(false);
//    node.calibrate(true);
    return 0;
}


