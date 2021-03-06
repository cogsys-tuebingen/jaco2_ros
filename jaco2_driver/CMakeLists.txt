cmake_minimum_required(VERSION 2.8.3)
project(jaco2_driver)

if(${TORQUE_EXPERT_MODE})
    message("Optionaly also gravity has to be compenstated.")
    add_definitions(-DTORQUE_EXPERT)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#add_definitions(-DCalib)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    trajectory_msgs
    dynamic_reconfigure
    rostest
    jaco2_data
    jaco2_msgs
    jaco2_msgs_conversion
    jaco2_kin_dyn_lib
    )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED VERSION 3.3.5)

link_directories(${PROJECT_SOURCE_DIR}/lib)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/jaco2_driver_configure.cfg
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES jaco2_driver
    CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs roscpp sensor_msgs std_msgs tf trajectory_msgs jaco2_msgs dynamic_reconfigure jaco2_data
    DEPENDS Boost Eigen3
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )
file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/jaco2_driver/*.hpp include/jaco2_driver/*.h
                                                                                   include/jaco2_driver/controller/*.hpp include/jaco2_driver/controller/*.h
                                                                                   include/jaco2_driver/data/*.hpp include/jaco2_driver/data/*.h
                                                                                   include/kinova/*.h)
## Declare a C++ library
add_library(jaco2_driver
    ${ALL_HEADERS}
    src/jaco2_driver.cpp
    src/jaco2_api.cpp
    src/jaco2_state.cpp
    src/data/jaco2_joint_state.cpp
    src/data/jaco2_kinova_conversion.cpp
    src/data/joint_state_outlier_filter.cpp
    src/data/gravity_estimator.cpp
    src/controller/collision_reaction.cpp
    src/controller/gripper_controller.cpp
    src/controller/torque_controller.cpp
    src/controller/trajectory_tracking_controller.cpp
    src/controller/p2p_joint_trajectory_controller.cpp
    src/controller/point_2_point_velocity_controller.cpp
    src/controller/torque_trajectory_controller.cpp
    src/controller/collision_repelling_p2p_controller.cpp
    src/controller/collision_repelling_p2p_torque_controller.cpp
    )

add_dependencies(jaco2_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(jaco2_driver
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    )

## Declare a C++ executable
add_executable(jaco2_driver_node src/nodes/jaco2_driver_node.cpp)
add_executable(gravity_parameter_estimation src/nodes/gravity_parameter_estimation)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(jaco2_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_driver_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(jaco2_driver_node
    jaco2_driver
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    )

target_link_libraries(gravity_parameter_estimation
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    jaco2_driver
    )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS jaco2_driver jaco2_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco2_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
