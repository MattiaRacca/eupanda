#ifndef PRIMITIVE_INTERFACE_H
#define PRIMITIVE_INTERFACE_H

// Generic includes
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

// ROS includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

// Franka includes
#include <franka_control/SetForceTorqueCollisionBehavior.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_msgs/FrankaState.h>
#include <franka_msgs/Errors.h>

// Custom services
#include "panda_pbd/EnableTeaching.h"
#include "panda_pbd/MoveToContactAction.h"
#include "panda_pbd/UserSyncAction.h"
#include "panda_pbd/MoveToEEAction.h"
#include "panda_pbd/MoveFingers.h"
#include "panda_pbd/ApplyForceFingers.h"
#include "panda_pbd/OpenGripper.h"
#include "panda_pbd/CloseGripper.h"

class PrimitiveInterface
{
private:
  ros::NodeHandle nh_;

  // const controller names
  const std::string IMPEDANCE_CONTROLLER = "cartesian_impedance_example_controller";

  // const frame names
  const std::string BASE_FRAME = "panda_link0";
  const std::string EE_FRAME = "panda_K";         // N.B.: panda_K and panda_EE are the same thing

  // Internal variables
  geometry_msgs::WrenchStamped last_wrench_;
  panda_pbd::MoveToContactFeedback move_to_contact_feedback_;
  panda_pbd::MoveToContactResult move_to_contact_result_;
  panda_pbd::MoveToEEFeedback move_to_ee_feedback_;
  panda_pbd::MoveToEEResult move_to_ee_result_;
  panda_pbd::UserSyncFeedback user_sync_feedback_;
  panda_pbd::UserSyncResult user_sync_result_;
  tf::TransformListener pose_listener_;
  boost::atomic_int interface_state_;  // 0: error, 1: ready, 2:busy

  // ROS SERVICES ====== servers
  ros::ServiceServer kinesthetic_server_;
  ros::ServiceServer move_fingers_server_;
  ros::ServiceServer apply_force_fingers_server_;

  // LEGACY SERVICES
  ros::ServiceServer open_gripper_server_;
  ros::ServiceServer close_gripper_server_;

  // ROS SERVICES ====== clients
  ros::ServiceClient cartesian_impedance_dynamic_reconfigure_client_;
  ros::ServiceClient forcetorque_collision_client_;
  ros::ServiceClient controller_manager_switch_;
  ros::ServiceClient controller_manager_list_;

  // ROS TOPICS ====== publishers
  ros::Publisher equilibrium_pose_publisher_;
  ros::Publisher interface_state_publisher_;
  // ROS TOPICS ====== subscribers
  ros::Subscriber franka_state_subscriber_;

  // TODO: current implementation of move_to_contact presents problem if we enable frames other than panda_link0
  // possible solution: save the goal pose in EE space?
  // other position: specify the frame of reference, as for the MoveToEE primitive?

  // ACTIONLIB ====== clients
  actionlib::SimpleActionClient<franka_gripper::GraspAction> *gripper_grasp_client_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> *gripper_move_client_;
  actionlib::SimpleActionClient<panda_pbd::MoveToEEAction> *move_to_ee_client_;
  actionlib::SimpleActionClient<panda_pbd::MoveToContactAction> *move_to_contact_client_;

  // ACTIONLIB ====== servers
  actionlib::SimpleActionServer<panda_pbd::MoveToContactAction> *move_to_contact_server_;
  actionlib::SimpleActionServer<panda_pbd::MoveToEEAction> *move_to_ee_server_;
  actionlib::SimpleActionServer<panda_pbd::UserSyncAction> *user_sync_server_;

  // Callbacks ====== services
  bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool moveFingersCallback(panda_pbd::MoveFingers::Request &req, panda_pbd::MoveFingers::Response &res);
  bool applyForceFingersCallback(panda_pbd::ApplyForceFingers::Request &req,
          panda_pbd::ApplyForceFingers::Response &res);

  // Callbacks ====== LEGACY services
  bool closeGripperCallback(panda_pbd::CloseGripper::Request &req, panda_pbd::CloseGripper::Response &res);
  bool openGripperCallback(panda_pbd::OpenGripper::Request &req, panda_pbd::OpenGripper::Response &res);

  // Callbacks ====== actionlib
  void moveToEECallback(const panda_pbd::MoveToEEGoalConstPtr &goal);
  void userSyncCallback(const panda_pbd::UserSyncGoalConstPtr &goal);
  void moveToContactCallback(const panda_pbd::MoveToContactGoalConstPtr &goal);

  // Callbacks ====== subscribers
  void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg);

  // Helper functions
  geometry_msgs::PoseStamped getPose(std::string origin, std::string destination);
  geometry_msgs::PoseStamped getEEPose();
  bool isInterfaceReady();
  bool adjustFTThreshold(double);
  bool adjustImpedanceControllerStiffness(double transl_stiff, double rotat_stiff, double ft_mult);
  bool adjustImpedanceControllerStiffness(geometry_msgs::PoseStamped desired_pose, double transl_stiff,
          double rotat_stiff, double ft_mult);
  bool adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req,
          panda_pbd::EnableTeaching::Response &res);
public:
  PrimitiveInterface();
};

#endif // PRIMITIVE_INTERFACE_H
