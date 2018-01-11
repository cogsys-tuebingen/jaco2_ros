#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#if ROS_VERSION_MINIMUM(1, 12, 0)
#include <moveit/move_group_interface/move_group_interface.h>
#else
#include <moveit/move_group_interface/move_group.h>
#endif

#include <fstream>
#include <iomanip>

class comparePlanner {
public:
    //    comparePlanner(std::string group, std::string description) {
    comparePlanner()
        :group_ ("manipulator"), startState(*group_.getCurrentState()), goalState(*group_.getCurrentState()) {
        //        std::string group = "manipulator";
        //        std::string description = "robot_description";
        //         moveit::planning_interface::MoveGroup group_ ("manipulator");

#if ROS_VERSION_MINIMUM(1, 12, 0)
        planningMonitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
#else
        planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
#endif
        jointNames_ =  planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();

        startState = planningMonitor_->getPlanningScene()->getCurrentState();
        goalState = planningMonitor_->getPlanningScene()->getCurrentState();

        group_.setStartStateToCurrentState();
        group_.setPlanningTime(1.0);
        //        startState = planningMonitor_->getPlanningScene()->getCurrentState();
        //        goalState = planningMonitor_->getPlanningScene()->getCurrentState();


    }

    void runPlanner(std::string currentPlanner, int iterations = 10) {
#if ROS_VERSION_MINIMUM(1, 12, 0)
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
#else
        moveit::planning_interface::MoveGroup::Plan my_plan_;
#endif

        iterations_ = iterations;

        group_.setPlannerId(currentPlanner);
        group_.setStartState(startState);


        //        std::pair<int, std::vector<double>> output;

        bool collision = false;
        bool succeeded = false;
        int numFailed = 0;
        int numSucceeded = 0;
        int numAttempts = 0;

        std::vector<double> planningTime(iterations_);
        for (int i = 0; i < iterations_; i++) {
            while(!succeeded) {
                goalState.setToRandomPositions();
                goalState.update();


                planning_scene_monitor::LockedPlanningSceneRW ps(planningMonitor_);
                ps->getCurrentStateNonConst().update();
                planning_scene::PlanningScenePtr scene = ps->diff();
                scene->decoupleParent();


                collision_detection::CollisionRequest collision_request;
                collision_detection::CollisionResult collision_result;
                collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

                collision_request.group_name = group_.getName();
                collision_result.clear();
                scene->setCurrentState(goalState);
                scene->checkCollision(collision_request, collision_result, goalState, acm);
                collision =  collision_result.collision;

                if(!collision) {
                    group_.setJointValueTarget(goalState);
                    succeeded = group_.plan(my_plan_).val == moveit_msgs::MoveItErrorCodes::SUCCESS;
                    if (!succeeded) {
                        numFailed ++;
                    } else {
                        planningTime[i] = my_plan_.planning_time_;
                        numSucceeded++;
                    }
                    numAttempts ++;
                }
            }
            collision = false;
            succeeded = false;
        }
        //        output.first = numFailed;
        //        output.second = planningTime;


        double sum = 0;
        for (int i = 0; i < planningTime.size(); i++) {
            sum += planningTime[i];
        }
        double average = sum/planningTime.size();

        //        int length = 0;
        //        for(; iterations != 0; iterations /= 10, length++);

        ROS_INFO_STREAM("Planner: " << currentPlanner << " Iterations: " << iterations_
                        << " Average Planning Time: " << average << " Success/Failure: " << numSucceeded << "/" << numFailed
                        << " Success Percentage: " << (numSucceeded/(double)numAttempts) *100 << "%"
                        << " Number of Attempts: " <<  numAttempts);
        std::ofstream outputFile;
        outputFile.open("/localhome/cgeckeler/tempdel99/plannerTimes.txt", std::ios::app);
        outputFile <<  "Planner: " << std::setiosflags(std::ios::left) << std::setw(25) << currentPlanner
                    << " Iterations: " << std::setiosflags(std::ios::left) << std::setw(10) << iterations_
                    << " Average Planning Time: " << std::setiosflags(std::ios::left) << std::fixed << std::setw(10) << average << std::scientific
                    << " Success Percentage: " <<  std::setiosflags(std::ios::left) << std::fixed << std::setprecision(0) << std::setw(3)<< (numSucceeded/(double)numAttempts) *100 << "%"
                    << " Success/Failure: " <<  numSucceeded << "/" <<  numFailed
                    << " Number of Attempts: " <<  numAttempts <<std::endl;
        //                   <<std::endl;
        outputFile.close();



        //        return output;
        //        sleep(2.0);

    }

private:
    std::vector<std::string> jointNames_;

#if ROS_VERSION_MINIMUM(1, 12, 0)
    moveit::planning_interface::MoveGroupInterface group_;
#else
    moveit::planning_interface::MoveGroup group_;
#endif
    //    moveit::planning_interface::MoveGroup::Plan my_plan_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;

    int iterations_;
    robot_state::RobotState startState;
    robot_state::RobotState goalState;

};








int main(int argc, char** argv) {
    ros::init (argc, argv, "comparePlanner");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //sleep(4.0);


    //  comparePlanner* plannerCompare = new comparePlanner();
    //    std::shared_ptr<comparePlanner> plannerCompare = std::shared_ptr<comparePlanner>(new comparePlanner());
    comparePlanner plannerCompare;

    sleep(2.0);
    //    moveit::planning_interface::MoveGroup group("manipulator");
    //    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    //    planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;
    //    planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    //    std::vector<std::string> jointNames_;
    //    jointNames_ =  planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();

    //    group.setStartStateToCurrentState();

    //    std::string currentPlanner = "RRTkConfigDefault";
    //    std::string currentPlanner = "TESTPLANNER";
    //    group.setPlannerId(currentPlanner);

    //    robot_state::RobotState state = planningMonitor_->getPlanningScene()->getCurrentStateNonConst();
    //    bool collision = false;
    //    bool succeeded = false;
    //    int numFailed = 0;
    //    moveit::planning_interface::MoveGroup::Plan my_plan;

    //    int iterations = 100;
    //    std::pair<int, std::vector<double>> output = runPlanner("RRTkConfigDefault", state, iterations, planningMonitor_, my_plan, group);
    //    std::vector<double> planningTime(iterations, output.second);
    //    int numFailed = output.first;

    //    double sum = 0;
    //    for (int i = 0; i < planningTime.size(); i++) {
    //        sum += planningTime[i];
    //    }
    //    double average = sum/planningTime.size();

    //    ROS_INFO_STREAM("Planner: " << currentPlanner << " Iterations: " << iterations << " Average Planning Time: " << average << " Success/Failure: " << iterations-numFailed << "/" << numFailed);

    //   std::string currentPlanner;
    //   currentPlanner = "RRTkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "RRTConnectkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "RRTStarkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);

    //   currentPlanner = "RRTkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "RRTConnectkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "RRTstarkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "TRRTkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "SBLkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "ESTkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "LBKPIECEkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "BKPIECEkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "KPIECEkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "PRMkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);
    //   currentPlanner = "PRMstarkConfigDefault";
    //   printOutput(runPlanner(currentPlanner, state, iterations, planningMonitor_, my_plan, group), currentPlanner, iterations);

    //  plannerCompare->runPlanner("RRTkConfigDefault");
    //  plannerCompare->runPlanner("RRTConnectkConfigDefault");
    //  plannerCompare->runPlanner("RRTstarkConfigDefault");
    //  plannerCompare->runPlanner("TRRTkConfigDefault");
    //  plannerCompare->runPlanner("SBLkConfigDefault");
    //  plannerCompare->runPlanner("ESTkConfigDefault");
    //  plannerCompare->runPlanner("LBKPIECEkConfigDefault");
    //  plannerCompare->runPlanner("BKPIECEkConfigDefault");
    //  plannerCompare->runPlanner("KPIECEkConfigDefault");
    //  plannerCompare->runPlanner("PRMkConfigDefault");
    //  plannerCompare->runPlanner("PRMstarkConfigDefault");

    plannerCompare.runPlanner("RRTkConfigDefault");
    plannerCompare.runPlanner("RRTConnectkConfigDefault");
    plannerCompare.runPlanner("RRTstarkConfigDefault");
    //    plannerCompare.runPlanner("TRRTkConfigDefault");
    plannerCompare.runPlanner("SBLkConfigDefault");
    plannerCompare.runPlanner("ESTkConfigDefault");
    plannerCompare.runPlanner("LBKPIECEkConfigDefault");
    plannerCompare.runPlanner("BKPIECEkConfigDefault");
    plannerCompare.runPlanner("KPIECEkConfigDefault");
    plannerCompare.runPlanner("PRMkConfigDefault");
    plannerCompare.runPlanner("PRMstarkConfigDefault");

    //    plannerCompare.runPlanner("TESTPLANNER");


    //    ros::Rate r(10);
    //  while(ros::ok()){
    //      ros::spinOnce();
    //      r.sleep();
    //  }



    ////#####################################################

    //    for (int i = 0; i < iterations; i++) {
    //        while(!succeeded) {
    //            state.setToRandomPositions();
    //            state.update();



    //            planning_scene_monitor::LockedPlanningSceneRW ps(planningMonitor_);
    //            ps->getCurrentStateNonConst().update();
    //            planning_scene::PlanningScenePtr scene = ps->diff();
    //            scene->decoupleParent();


    //            collision_detection::CollisionRequest collision_request;
    //            collision_detection::CollisionResult collision_result;
    //            collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();

    //            collision_request.group_name = group.getName();
    //            collision_result.clear();
    //            scene->setCurrentState(state);
    //            scene->checkCollision(collision_request, collision_result, state, acm);
    //            collision =  collision_result.collision;

    //            if(!collision) {
    //                group.setJointValueTarget(state);
    //                succeeded = group.plan(my_plan);
    //                if (!succeeded) {
    //                    numFailed ++;
    //                } else
    //                    planningTime[i] = my_plan.planning_time_;
    //            }
    //        }
    //    }

    ////#####################################################



    //    group.setJointValueTarget(state);

    //    bool success = group.plan(my_plan);

    //    ROS_INFO("Visualizing plan 1 (pose goal) %s:",succeeded?"SUCCESS":"FAILED");
    //    ROS_INFO_STREAM("Planning took: "<< my_plan.planning_time_);

    /* Sleep to give Rviz time to visualize the plan. */
    //    sleep(5.0);


    /*
    double sum = 0;
    for (int i = 0; i < planningTime.size(); i++) {
        sum += planningTime[i];
    }
    double average = sum/planningTime.size();

    ROS_INFO_STREAM("Planner: " << currentPlanner << " Iterations: " << iterations << " Average Planning Time: " << average << " Success/Failure: " << iterations-numFailed << "/" << numFailed);
*/

    return 0;
}
