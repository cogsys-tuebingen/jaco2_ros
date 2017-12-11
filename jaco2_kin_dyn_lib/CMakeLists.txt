cmake_minimum_required(VERSION 2.8.3)
project(jaco2_kin_dyn_lib)

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

find_package(catkin REQUIRED COMPONENTS
#  kdl_parser
  roscpp
  rostest
  tf
  tf_conversions
  trac_ik_lib
  rosbag
  jaco2_msgs
  jaco2_data
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jaco2_kin_dyn_lib
  CATKIN_DEPENDS kdl_parser roscpp tf trac_ik_lib jaco2_data
  DEPENDS system_lib orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
file(GLOB_RECURSE ALL_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp include/*.h)

add_library(jaco2_kin_dyn_lib
   src/kdl_joint_state_data.cpp
   src/jaco2_modified_dynamic_model.cpp
   src/jaco2_dynamic_model.cpp
   src/jaco2_kinematic_model.cpp
   src/yaml_to_kdl_tranform.cpp
   src/jaco2_residual_vector.cpp
   src/kdl_conversion.cpp
   src/joint_vel_pos_estimator.cpp
   src/jaco2_residual_kalman.cpp
   ${ALL_HEADERS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(jaco2_kin_dyn_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(jaco2_tf_static_transforms_conversion src/jaco2_tf_static_tranforms_conversion.cpp)
add_executable(jaco2_kin_dyn_lib_test test/test_jaco2_kin_dyn.cpp)
add_executable(jaco2_modified_dyn_lib_test test/test_jaco2_modified_dyn_model.cpp)
add_executable(jaco2_estimation_test test/test_pos_vel_estimation.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(jaco2_tf_static_transforms_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} jaco2_kin_dyn_lib)
add_dependencies(jaco2_kin_dyn_lib_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GTEST_LIBRARIES} jaco2_kin_dyn_lib)
add_dependencies(jaco2_modified_dyn_lib_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GTEST_LIBRARIES} jaco2_kin_dyn_lib)

## Specify libraries to link a library or executable target against
target_link_libraries(jaco2_kin_dyn_lib
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   yaml-cpp
)

target_link_libraries(jaco2_tf_static_transforms_conversion
   ${catkin_LIBRARIES}
   jaco2_kin_dyn_lib
)

target_link_libraries(jaco2_kin_dyn_lib_test
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${GTEST_LIBRARIES}
   jaco2_kin_dyn_lib
)

target_link_libraries(jaco2_modified_dyn_lib_test
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${GTEST_LIBRARIES}
   jaco2_kin_dyn_lib
)


target_link_libraries(jaco2_estimation_test
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${GTEST_LIBRARIES}
   jaco2_kin_dyn_lib
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
# install(TARGETS jaco2_kin_dyn jaco2_kin_dyn_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco2_kin_dyn.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
