#include "static_data_generator.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

//void spinThread()
//{
//    ros::spin();
//}

StaticDataGenerator* node_ptr;

void mySigintHandler(int sig)
{
    if(node_ptr){
        node_ptr->saveBag();
    }
    ROS_INFO_STREAM("Ctrl-C caught, closing bag.");
    ros::shutdown();
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "static_data_generator_node");

    ros::NodeHandle nh("~");

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = mySigintHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    StaticDataGenerator node(nh);
    node_ptr = &node;

    //    ros::MultiThreadedSpinner mspin(2);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO_STREAM("Start");

//    boost::thread spin_thread(&spinThread);
    node.generateData();
    spinner.stop();

    std::cout << "DONE !!!" << std::endl;
    ROS_INFO_STREAM("DONE !!!");

    ros::shutdown();

    return 0;
}
