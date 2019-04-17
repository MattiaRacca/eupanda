#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

// Generic includes
#include <boost/array.hpp>

// ROS includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>

// Franka includes
#include <franka_control/SetForceTorqueCollisionBehavior.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

// Custom services
#include "panda_pbd/EnableTeaching.h"

class DemoInterface
{
private:
  ros::NodeHandle nh_;

  // Services (servers and clients)
  ros::ServiceServer kinesthetic_server_;
  ros::ServiceServer grasp_server_;
  ros::ServiceClient cartesian_impedance_dynamic_reconfigure_client_;
  ros::ServiceClient forcetorque_collision_client_;

  // Topics (publishers and subscribers)
  ros::Publisher equilibrium_pose_publisher_;

  // Action (servers and clients)
  actionlib::SimpleActionClient<franka_gripper::GraspAction> *gripper_grasp_client_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> *gripper_move_client_;

  // TF
  tf::TransformListener pose_listener_;

  // Callbacks
  bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool graspCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  // Helper functions
  geometry_msgs::PoseStamped getEEPose();
  bool AdjustFTThreshold(double);
public:
  DemoInterface();
};

#endif // DEMO_INTERFACE_H