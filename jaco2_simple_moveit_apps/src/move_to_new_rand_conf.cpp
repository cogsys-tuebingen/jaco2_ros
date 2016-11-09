#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>

struct Configuration{

    bool equal(Configuration& other, std::vector<double>& threshold)
    {
        if(other.angles.size() == angles.size()){
            bool eq = true;
            for(std::size_t i = 0; i < angles.size(); ++i){
                double diff = std::abs(angles[i] - other.angles[i]);
                eq &= diff < threshold;
            }
            return eq;
        }
        else{
            return false;
        }
    }

    std::string to_string()
    {
        std::string res;
        for(auto val : angles){
            res += std::to_string(val) + ";";
        }
    }

    std::vector<double> angles;
};

struct ConfigurationList{

    bool contains(Configurations& conf)
    {
        if(conf.angles.size() == jointNames.size()){
            bool test = false;
            for(auto elemets : configurations){
                test |= elemets.equal(conf, offsets);
                if(test){
                    return test;
                }
            }
            else return test;
        }
        else{
            return false;
        }
    }

    void save(std::string filename)
    {
          std::ofstream file(filename);

          for()
    }

    void load(std::string filename)
    {

    }

    std::size_t n_joints_;
    std::vector<double> offsets;
    std::vector<std::string> jointNames;
    std::vector<Configuration> configurations;
};

class NewRandomConfPlanner{
public:
    NewRandomConfPlanner(std::string group, std::string description, std::string planner) :
        group_(group)
    {
        planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(description);
        configurations_.jointNames =  planningMonitor_->getRobotModel()->getJointModelGroup("manipulator")->getActiveJointModelNames();
        planningMonitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        group_.setPlannerId(planner);
        group_.setStartStateToCurrentState();
        group_.setPlanningTime(5.0);

    }


    void move2NewConf()
    {
        Configuration rand;
        rand.angles = group.getRandomJointValues();

        robot_state::RobotState& random_state = planningMonitor_->getPlanningScene()->getCurrentStateNonConst();


        for(std::size_t i = 0; i < configurations_.jointNames.size(); ++i){
            random_state.setJointPositions(configurations_.jointNames[i], &(jvalues[i]));
        }

        random_state.update();

        collision = checkCollision(planningMonitor_,random_state);
        ROS_INFO_STREAM("Random state is "
                        << (collision ? "in" : "not in")
                        << " collision" );


        if(!collision && !configurations_.contains(rand)) {


            moveit::planning_interface::MoveGroup::Plan my_plan;
            group.setJointValueTarget(jvalues);
            //                    moveGroup_.setStartStateToCurrentState();
            group.setPlanningTime(3.0);
            moveit_msgs::MoveItErrorCodes success = group.plan(my_plan);

            //                    std::cout << "Test 4: Planning successfull:  "
            //                              << "error code: " << success.val << std::endl;
            if(success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                //                        Group.execute(my_plan);
                success = group.move();
            }
            if(success.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                collision = true;
            }

        }

        private:

        moveit::planning_interface::MoveGroup group_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        planning_scene_monitor::PlanningSceneMonitorPtr planningMonitor_;

        ConfigurationList configurations_;
    };

    int main(int argc, char *argv[])
    {
        ros::init (argc, argv, "home_arm");
        ros::NodeHandle node_handle("~");
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::Duration sleep_time(1.0);
        sleep_time.sleep();
        sleep_time.sleep();

        std::vector<double> offsets;




        //    ros::shutdown();
        return 0;
    }
