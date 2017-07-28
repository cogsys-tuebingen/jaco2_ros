cmake_minimum_required(VERSION 2.8.3)
project(jaco2_calibration_utils)

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

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  jaco2_data
  jaco2_kin_dyn_lib
  jaco2_msgs
  jaco2_msgs_conversion
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  tf
  rosbag
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(NLOPT REQUIRED)


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
  LIBRARIES jaco2_calibration_utils
  CATKIN_DEPENDS jaco2_data jaco2_kin_dyn_lib jaco2_msgs roscpp sensor_msgs tf
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
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)
## Declare a C++ library
add_library(jaco2_calibration_utils
    include/jaco2_calibration_utils/dynamic_calibration_sample.hpp
    include/jaco2_calibration_utils/acceleration_samples.hpp
    include/jaco2_calibration_utils/dynamic_calibrated_parameters.hpp
    src/dynamic_residual.cpp
    src/jaco2_calibration_io.cpp
    src/static_data_generator.cpp
    ${ALL_HEADERS}
)

add_executable(jaco2_create_calib_data_node src/create_calib_data.cpp)
add_executable(jaco2_rigid_body_regression src/jaco2_rigid_body_regression.cpp)
add_executable(record_calibration_data src/record_calibration_data.cpp)
add_executable(jaco2_rigid_body_subplex src/jaco2_dyn_subplex_calibration.cpp)
add_executable(record_static_data src/record_static_data.cpp)
add_executable(calib_joint_velocity src/calib_joint_velocity.cpp)
add_executable(jaco2_torque_offset_calibration src/jaco2_torque_offsets_sinus_fit.cpp)
add_executable(jaco2_static_data_generartion_node src/static_data_generation_node.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(jaco2_calibration_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco2_rigid_body_regression ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration_utils)
add_dependencies(jaco2_create_calib_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration_utils)
## Specify libraries to link a library or executable target against

target_link_libraries(jaco2_calibration_utils
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${NLOPT_LIBRARIES}
  yaml-cpp
)



 target_link_libraries(jaco2_rigid_body_regression
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   jaco2_calibration_utils
)



target_link_libraries(jaco2_create_calib_data_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   jaco2_calibration_utils
)

target_link_libraries(record_calibration_data
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   jaco2_calibration_utils
)

target_link_libraries(jaco2_rigid_body_subplex
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    jaco2_calibration_utils
    ${NLOPT_LIBRARIES}
)

target_link_libraries(record_static_data
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   jaco2_calibration_utils
)

target_link_libraries(calib_joint_velocity
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   jaco2_calibration_utils
)

target_link_libraries(jaco2_torque_offset_calibration
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   jaco2_calibration_utils
)

target_link_libraries(jaco2_static_data_generartion_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    jaco2_calibration_utils
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
# install(TARGETS jaco2_calibration_utils jaco2_calibration_utils_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco2_calibration_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
