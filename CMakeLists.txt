cmake_minimum_required(VERSION 2.8.3)
project(panda_pbd)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  message_generation
  actionlib_msgs
  franka_gripper
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_service_files(
  FILES
  EnableTeaching.srv
  MoveFingers.srv
  ApplyForceFingers.srv
  OpenGripper.srv
  CloseGripper.srv
)

add_action_files(
  DIRECTORY action
  FILES MoveToContact.action UserSync.action MoveToEE.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES panda_pbd
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs actionlib_msgs 
#    DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(primitive_interface src/primitive_interface_node.cpp
        src/primitive_interface.cpp)

target_link_libraries(primitive_interface
   ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/pbd_interface_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
