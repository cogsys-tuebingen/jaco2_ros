cmake_minimum_required(VERSION 2.8.3)
project(jaco2_calibration)

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
  jaco2_driver
  jaco2_kin_dyn_lib
  jaco2_msgs
  jaco2_calibration_utils
  roscpp
  sensor_msgs
  tf
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
)



## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Ceres QUIET)
find_package(imu_tk QUIET)
find_package(Boost REQUIRED)

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
  LIBRARIES jaco2_calibration
  CATKIN_DEPENDS jaco2_driver jaco2_kin_dyn_lib jaco2_msgs jaco2_calibration_utils roscpp sensor_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${imu_tk_INCLUDE_DIRS}
)


if(${Ceres_FOUND} AND ${imu_tk_FOUND})

    add_library(jaco2_calibration
        include/jaco2_calibration/com_inertia_residuals.hpp
        include/jaco2_calibration/manipulator_dynamic_residuals.hpp
        src/jaco2_calibration.cpp
    )

    add_executable(jaco2_calibration_node src/jaco2_calibration_node.cpp)
    add_executable(jaco2_file_calibration_node src/calibrate_from_file.cpp)
    add_executable(jaco2_simulate_calibration src/simulate_data.cpp)


    add_dependencies(jaco2_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(jaco2_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration)
    add_dependencies(jaco2_file_calibration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration)
    add_dependencies(jaco2_simulate_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration)

    target_link_libraries(jaco2_calibration
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
      ${Boost_LIBRARIES}
      ${imu_tk_LIBRARIES}
      yaml-cpp
    )

    target_link_libraries(jaco2_calibration_node
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
      ${Boost_LIBRARIES}
      ${imu_tk_LIBRARIES}
      jaco2_calibration
    )

    target_link_libraries(jaco2_file_calibration_node
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
      ${Boost_LIBRARIES}
      ${imu_tk_LIBRARIES}
      jaco2_calibration
    )

    target_link_libraries(jaco2_simulate_calibration
      ${catkin_LIBRARIES}
      ${CERES_LIBRARIES}
      ${Boost_LIBRARIES}
      ${imu_tk_LIBRARIES}
      jaco2_calibration
    )
else(${CERES_FOUND} AND ${imu_tk_FOUND})
    message("Optional package jaco2_calibration is not going to be build, since ceres and imu_tk is not found.")

endif()



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
# install(TARGETS jaco2_calibration jaco2_calibration_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco2_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
