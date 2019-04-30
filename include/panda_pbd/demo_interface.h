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

// Custom services
#include "panda_pbd/EnableTeaching.h"
#include "panda_pbd/UserSync.h"
#include "panda_pbd/MoveToContactAction.h"

class DemoInterface
{
private:
  ros::NodeHandle nh_;

  // const controller names
  const std::string impedance_controller = "cartesian_impedance_example_controller";
  const std::string direction_controller = "cartesian_impedance_direction_controller";

  // const frame names
  const std::string base_frame = "panda_link0";
  const std::string ee_frame = "panda_K";

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
  bool adjustFTThreshold(double);
  bool adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
  bool adjustImpedanceControllerStiffness(double transl_stiff, double rotat_stiff, double ft_mult);

  bool adjustDirectionControllerParameters(geometry_msgs::Vector3 direction, double speed, double transl_stiff,
          double rotat_stiff, double ft_mult);
public:
  DemoInterface();
};

#endif // DEMO_INTERFACE_H