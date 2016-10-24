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


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  jaco2_driver
  jaco2_kin_dyn_lib
  jaco2_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
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
  CATKIN_DEPENDS jaco2_driver jaco2_kin_dyn_lib jaco2_msgs roscpp sensor_msgs tf
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
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(jaco2_calibration_utils
    include/jaco2_calibration_utils/dynamic_calibration_sample.hpp
    include/jaco2_calibration_utils/acceleration_samples.hpp
    include/jaco2_calibration_utils/jaco2_calibration_io.hpp
    include/jaco2_calibration_utils/dynamic_calibrated_parameters.hpp
)

add_executable(jaco2_create_calib_data_node src/create_calib_data.cpp)
add_executable(jaco2_rigid_body_regression src/jaco2_rigid_body_regression.cpp)
add_executable(record_calibration_data src/record_calibration_data.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(jaco2_calibration_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco2_rigid_body_regression ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration)
add_dependencies(jaco2_create_calib_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_calibration)
## Specify libraries to link a library or executable target against

target_link_libraries(jaco2_calibration_utils
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)



 target_link_libraries(jaco2_rigid_body_regression
   jaco2_calibration_utils
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)



target_link_libraries(jaco2_create_calib_data_node
   jaco2_calibration_utils
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(record_calibration_data
   jaco2_calibration_utils
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
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
