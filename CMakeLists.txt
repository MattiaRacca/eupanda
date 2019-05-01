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
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  EnableTeaching.srv
)

add_action_files(
  DIRECTORY action
  FILES MoveToContact.action UserSync.action
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

add_executable(demo_interface src/demo_interface_node.cpp
        src/demo_interface.cpp)

target_link_libraries(demo_interface
   ${catkin_LIBRARIES}
)
