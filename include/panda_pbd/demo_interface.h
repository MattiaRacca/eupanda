#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

// Generic includes
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>

// ROS includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>

// Franka includes
#include <franka_control/SetForceTorqueCollisionBehavior.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>

// Custom services
#include "panda_pbd/EnableTeaching.h"
#include "panda_pbd/UserSync.h"
#include "panda_pbd/MoveToContactAction.h"

class DemoInterface
{
private:
  ros::NodeHandle nh_;

  // const controller names
  const std::string IMPEDANCE_CONTROLLER = "cartesian_impedance_example_controller";
  const std::string DIRECTION_CONTROLLER = "cartesian_impedance_direction_controller";
  const std::string JOINT_CONTROLLER = "position_joint_trajectory_controller";

  // const frame names
  const std::string BASE_FRAME = "panda_link0";
  const std::string EE_FRAME = "panda_K";
  const std::string HAND_FRAME = "panda_hand";

  // MoveIt! stuff
  const std::string PLANNING_GROUP = "panda_arm_hand";
  const robot_state::JointModelGroup *joint_model_group;
  moveit::planning_interface::MoveGroupInterface *move_group_;

  // Internal variable
  geometry_msgs::WrenchStamped last_wrench_;
  panda_pbd::MoveToContactFeedback move_to_contact_feedback_;
  panda_pbd::MoveToContactResult move_to_contact_result_;

  // Services (servers and clients)
  ros::ServiceServer kinesthetic_server_;
  ros::ServiceServer grasp_server_;
  ros::ServiceServer user_sync_server_;
  ros::ServiceClient cartesian_impedance_dynamic_reconfigure_client_;
  ros::ServiceClient cartesian_impedance_direction_dynamic_reconfigure_client_;
  ros::ServiceClient forcetorque_collision_client_;
  ros::ServiceClient controller_manager_switch_;

  // Topics (publishers and subscribers)
  ros::Publisher equilibrium_pose_publisher_;

  // Action (servers and clients)
  actionlib::SimpleActionClient<franka_gripper::GraspAction> *gripper_grasp_client_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> *gripper_move_client_;
  actionlib::SimpleActionServer<panda_pbd::MoveToContactAction> *move_to_contact_server_;

  // TF
  tf::TransformListener pose_listener_;

  // Callbacks
  bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool graspCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool userSyncCallback(panda_pbd::UserSyncRequest &req, panda_pbd::UserSyncResponse &res);
  void moveToContactCallback(const panda_pbd::MoveToContactGoalConstPtr &goal);

  // Helper functions
  geometry_msgs::PoseStamped getEEPose();
  geometry_msgs::PoseStamped getHandPose();
  bool adjustFTThreshold(double);
  bool adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool adjustImpedanceControllerStiffness(double transl_stiff, double rotat_stiff, double ft_mult);

  bool adjustDirectionControllerParameters(geometry_msgs::Vector3 direction, double speed, double transl_stiff,
          double rotat_stiff, double ft_mult);

  bool testPlanning();
public:
  DemoInterface();
};

#endif // DEMO_INTERFACE_H