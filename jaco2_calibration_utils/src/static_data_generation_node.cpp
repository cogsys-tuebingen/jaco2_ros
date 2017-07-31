#include "static_data_generator.h"

//void spinThread()
//{
//    ros::spin();
//}

//std::shared_ptr<StaticDataGenerator> node_ptr;

//void mySigintHandler(int sig)
//{
//    if(node_ptr){
//        node_ptr->saveBag();
//    }
//    ros::shutdown();
//}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "static_data_generator_node");

    ros::NodeHandle nh("~");

    StaticDataGenerator node(nh);
//    node_ptr = std::make_shared<StaticDataGenerator>(node);

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
