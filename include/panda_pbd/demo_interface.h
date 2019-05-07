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
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_manager_msgs/SwitchController.h>

// Franka includes
#include <franka_control/SetForceTorqueCollisionBehavior.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>

// Custom services
#include "panda_pbd/EnableTeaching.h"
#include "panda_pbd/OpenGripper.h"
#include "panda_pbd/CloseGripper.h"
#include "panda_pbd/MoveToContactAction.h"
#include "panda_pbd/UserSyncAction.h"

class DemoInterface
{
private:
  ros::NodeHandle nh_;

  // const controller names
  const std::string IMPEDANCE_CONTROLLER = "cartesian_impedance_example_controller";
  const std::string IMPEDANCE_DIRECTION_CONTROLLER = "cartesian_impedance_direction_controller";
  const std::string IMPEDANCE_TRAJECTORY_CONTROLLER = "cartesian_impedance_trajectory_controller";
  const std::string JOINT_CONTROLLER = "position_joint_trajectory_controller";

  // const frame names
  const std::string BASE_FRAME = "panda_link0";
  const std::string EE_FRAME = "panda_K";         // N.B.: panda_K and panda_EE are the same thing
  const std::string LINK8_FRAME = "panda_link8";  // Used as EE for the panda_arm planning group by MoveIt!

  // MoveIt! stuff
  const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface *move_group_;

  // Internal variable
  geometry_msgs::WrenchStamped last_wrench_;
  panda_pbd::MoveToContactFeedback move_to_contact_feedback_;
  panda_pbd::MoveToContactResult move_to_contact_result_;
  panda_pbd::UserSyncFeedback user_sync_feedback_;
  panda_pbd::UserSyncResult user_sync_result_;

  // ROS SERVICES ====== servers
  ros::ServiceServer kinesthetic_server_;
  ros::ServiceServer close_gripper_server_;
  ros::ServiceServer open_gripper_server_;
  ros::ServiceServer moveit_test_server_;

  // ROS SERVICES ====== clients
  ros::ServiceClient cartesian_impedance_dynamic_reconfigure_client_;
  ros::ServiceClient cartesian_impedance_direction_dynamic_reconfigure_client_;
  ros::ServiceClient forcetorque_collision_client_;
  ros::ServiceClient controller_manager_switch_;

  // Topics (publishers and subscribers)
  ros::Publisher equilibrium_pose_publisher_;
  ros::Publisher target_pose_publisher_;

  // Action (servers and clients)
  actionlib::SimpleActionClient<franka_gripper::GraspAction> *gripper_grasp_client_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> *gripper_move_client_;
  actionlib::SimpleActionServer<panda_pbd::MoveToContactAction> *move_to_contact_server_;
  actionlib::SimpleActionServer<panda_pbd::UserSyncAction> *user_sync_server_;

  // TF
  tf::TransformListener pose_listener_;

  // Callbacks
  bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool closeGripperCallback(panda_pbd::CloseGripper::Request &req, panda_pbd::CloseGripper::Response &res);
  bool openGripperCallback(panda_pbd::CloseGripper::Request &req, panda_pbd::CloseGripper::Response &res);
  bool moveitTestCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  bool moveToEETestCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  void userSyncCallback(const panda_pbd::UserSyncGoalConstPtr &goal);
  void moveToContactCallback(const panda_pbd::MoveToContactGoalConstPtr &goal);

  // Helper functions
  geometry_msgs::PoseStamped getPose(std::string origin, std::string destination);
  geometry_msgs::PoseStamped getEEPose();
  bool adjustFTThreshold(double);
  bool adjustImpedanceControllerStiffness(double transl_stiff, double rotat_stiff, double ft_mult);
  bool adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool adjustDirectionControllerParameters(geometry_msgs::Vector3 direction, double speed, double transl_stiff,
          double rotat_stiff, double ft_mult);
public:
  DemoInterface();
};

#endif // DEMO_INTERFACE_H