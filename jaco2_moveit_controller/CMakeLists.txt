cmake_minimum_required(VERSION 2.8.3)
project(jaco2_moveit_controller)

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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    jaco2_msgs
    jaco2_utils
    tf
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface

    actionlib_msgs
    )

message("################################### ROS VERSION: ${roscpp_VERSION_MAJOR}.${roscpp_VERSION_MINOR}.${roscpp_VERSION_PATCH}")

    ## System dependencies are found with CMake's conventions
    # find_package(Boost REQUIRED COMPONENTS system)

#    add_action_files(
#        DIRECTORY action
#        FILES)

#    generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

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
        #  INCLUDE_DIRS include
        LIBRARIES jaco2_moveit_controller
        CATKIN_DEPENDS moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface roscpp jaco2_msgs jaco2_utils actionlib_msgs message_runtime
        DEPENDS system_lib
        )

    ###########
    ## Build ##
    ###########

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    # include_directories(include)
    include_directories(
        ${catkin_INCLUDE_DIRS}
        curses
        )

#    # Declare a C++ library
#     add_library(jaco2_moveit_controller

#     )

    ## Declare a C++ executable
    add_executable(gripper_action_command src/gripper_action_command.cpp)

    ## Add cmake target dependencies of the executable
    ## same as for the library above
    add_dependencies(gripper_action_command
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(gripper_action_command
        ${catkin_LIBRARIES}
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
    # install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco2_moveit_controller.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
