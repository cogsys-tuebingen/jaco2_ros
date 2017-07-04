cmake_minimum_required(VERSION 2.8.3)
project(jaco2_msgs)

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
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  std_srvs
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   JointVelocity.msg
   JointAngles.msg
   FingerPosition.msg
   Jaco2Sensor.msg
   Jaco2Acc.msg
   Jaco2JointState.msg
   Jaco2Accelerometers.msg
   Jaco2GfreeTorques.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   HomeArm.srv
   SetForceControlParams.srv
   Start.srv
   Stop.srv
   CalibAcc.srv
   SetTorqueZero.srv
   CartesianTransformation.srv
   SetSampledConfigService.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   ArmJointAngles.action
   SetFingersPosition.action
   GripperControl.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation actionlib actionlib_msgs geometry_msgs message_runtime std_msgs
)

