cmake_minimum_required(VERSION 2.8.3)
project(jaco2_driver)

#set(CMAKE_BUILD_TYPE Debug)

## Enforce that we use C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
    add_definitions(-std=c++0x)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

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
    jaco2_msgs
    dynamic_reconfigure
    jaco2_kin_dyn_lib
    rostest

    )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


link_directories(${PROJECT_SOURCE_DIR}/lib)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder

generate_dynamic_reconfigure_options(
    cfg/jaco2_driver_configure.cfg
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES jaco2_driver
    CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs roscpp sensor_msgs std_msgs tf trajectory_msgs jaco2_msgs dynamic_reconfigure
    DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )
file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/jaco2_driver/*.hpp include/jaco2_driver/*.h
                                                                                   include/jaco2_driver/controller/*.hpp include/jaco2_driver/controller/*.h)
## Declare a C++ library
add_library(jaco2_driver
    ${ALL_HEADERS}
    src/jaco2_driver.cpp
    src/jaco2_api.cpp
    src/jaco2_state.cpp
    src/controller/collision_reaction.cpp
    src/controller/gripper_controller.cpp
    src/controller/torque_controller.cpp
    src/controller/p2p_joint_trajectory_controller.cpp
    src/controller/point_2_point_velocity_controller.cpp
    src/controller/torque_trajectory_controller.cpp
    src/controller/collision_repelling_p2p_controller.cpp
    src/controller/collision_repelling_p2p_torque_controller.cpp
    )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(jaco2_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(jaco2_driver
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    )

## Declare a C++ executable
add_executable(jaco2_driver_node src/nodes/jaco2_driver_node.cpp)
add_executable(gravity_parameter_estimation src/nodes/gravity_parameter_estimation)
add_executable(test_torque_lut test/test_torque_offset_lut.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(jaco2_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_driver_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(jaco2_driver_node
    jaco2_driver
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

target_link_libraries(gravity_parameter_estimation
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    jaco2_driver
    )

target_link_libraries(test_torque_lut
    jaco2_driver
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GTEST_LIBRARIES}
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
