#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface(): nh_("~")
{
  // Service servers
  kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
  grasp_server_ = nh_.advertiseService("grasp", &DemoInterface::graspCallback, this);
  user_sync_server_ = nh_.advertiseService("user_sync", &DemoInterface::userSyncCallback, this);

  equilibrium_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose", 10);

  // Clients
  cartesian_impedance_dynamic_reconfigure_client_ = nh_.
      serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");
  forcetorque_collision_client_ = nh_.
                                  serviceClient<franka_control::SetForceTorqueCollisionBehavior>
                                          ("/franka_control/set_force_torque_collision_behavior");
  gripper_grasp_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", true);
  gripper_move_client_ = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("/franka_gripper/move", true);
}

geometry_msgs::PoseStamped DemoInterface::getEEPose()
{
  std::string tf_err_msg;
  std::string refFrame = "panda_link0";
  std::string childFrame = "panda_K";
  tf::StampedTransform transform;

  if (!pose_listener_.waitForTransform(refFrame, childFrame, ros::Time(0),
                                       ros::Duration(0.5), ros::Duration(0.01),
                                       &tf_err_msg))
  {
    ROS_ERROR_STREAM("Unable to get pose from TF: " << tf_err_msg);
  }
  else
  {
    try
    {
      pose_listener_.lookupTransform(refFrame, childFrame,
                                     ros::Time(0), // get latest available
                                     transform);
    }
    catch (const tf::TransformException &e)
    {
      ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in "
                       << refFrame);
    }
  }

  geometry_msgs::TransformStamped current_ee_transform;
  tf::transformStampedTFToMsg(transform, current_ee_transform);

  geometry_msgs::PoseStamped current_ee_pose;
  current_ee_pose.header.frame_id = current_ee_transform.header.frame_id;
  current_ee_pose.pose.orientation = current_ee_transform.transform.rotation;
  current_ee_pose.pose.position.x = current_ee_transform.transform.translation.x;
  current_ee_pose.pose.position.y = current_ee_transform.transform.translation.y;
  current_ee_pose.pose.position.z = current_ee_transform.transform.translation.z;

  return current_ee_pose;
}

bool DemoInterface::adjustFTThreshold(double ft_multiplier)
{
  franka_control::SetForceTorqueCollisionBehavior collision_srv;

  boost::array<double, 6> force_threshold{ {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} };
  boost::array<double, 7> torque_threshold{ {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} };

  if (ft_multiplier <= 1.0){
    ROS_ERROR("ForceTorque Multiplier has to be greater than 1...");
    return false;
  }

  for (auto& value : force_threshold)
  {
    value *= ft_multiplier;
  }
  for (auto& value : torque_threshold)
  {
    value *= ft_multiplier;
  }

  collision_srv.request.lower_force_thresholds_nominal = force_threshold;
  collision_srv.request.upper_force_thresholds_nominal = force_threshold;
  collision_srv.request.lower_torque_thresholds_nominal = torque_threshold;
  collision_srv.request.upper_torque_thresholds_nominal = torque_threshold;

  forcetorque_collision_client_.call(collision_srv);
}

bool DemoInterface::adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req,
                                                       panda_pbd::EnableTeaching::Response &res)
                                                       {
  dynamic_reconfigure::Reconfigure stiffness_srv;

  dynamic_reconfigure::DoubleParameter translational_stiff;
  dynamic_reconfigure::DoubleParameter rotational_stiff;
  translational_stiff.name = "translational_stiffness";
  rotational_stiff.name = "rotational_stiffness";

  double default_translation_stiffness = 200.0;
  double default_rotational_stiffness = 10.0;

  switch (req.teaching) {
    case 0: {
      ROS_INFO("Teaching mode deactivated...");
      adjustFTThreshold(1.0);
      translational_stiff.value = default_translation_stiffness;
      rotational_stiff.value = default_rotational_stiffness;
      break;
    }
    case 1: {
      ROS_INFO("Full Teaching mode activated...");
      adjustFTThreshold(req.ft_threshold_multiplier);
      translational_stiff.value = 0.0;
      rotational_stiff.value = 0.0;
      break;
    }
    case 2: {
      ROS_INFO("Position Teaching mode activated...");
      adjustFTThreshold(req.ft_threshold_multiplier);
      translational_stiff.value = 0.0;
      rotational_stiff.value = default_rotational_stiffness;
      break;
    }
    case 3: {
      ROS_INFO("Orientation Teaching mode activated...");
      adjustFTThreshold(req.ft_threshold_multiplier);
      translational_stiff.value = default_translation_stiffness;
      rotational_stiff.value = 0.0;
      break;
    }
    default: {
      ROS_ERROR("Unknown value of teaching - nothing happened");
      return false;
    }
  }

  stiffness_srv.request.config.doubles.push_back(translational_stiff);
  stiffness_srv.request.config.doubles.push_back(rotational_stiff);

  return cartesian_impedance_dynamic_reconfigure_client_.call(stiffness_srv);
}

bool DemoInterface::adjustImpedanceControllerStiffness(double transl_stiff, double rotat_stiff, double ft_mult) {
  dynamic_reconfigure::Reconfigure stiffness_srv;

  dynamic_reconfigure::DoubleParameter translational_stiff;
  dynamic_reconfigure::DoubleParameter rotational_stiff;
  translational_stiff.name = "translational_stiffness";
  rotational_stiff.name = "rotational_stiffness";

  double default_translation_stiffness = transl_stiff;
  double default_rotational_stiffness = rotat_stiff;

  ROS_INFO("Changing Impedance Controller Stiffness...");
  adjustFTThreshold(ft_mult);
  translational_stiff.value = default_translation_stiffness;
  rotational_stiff.value = default_rotational_stiffness;

  stiffness_srv.request.config.doubles.push_back(translational_stiff);
  stiffness_srv.request.config.doubles.push_back(rotational_stiff);

  return cartesian_impedance_dynamic_reconfigure_client_.call(stiffness_srv);
}

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
    panda_pbd::EnableTeaching::Response &res)
{
  res.ee_pose = getEEPose();
  equilibrium_pose_publisher_.publish(res.ee_pose);
  ros::Duration(0.5).sleep(); // to allow the controller to receive the now equilibrium pose
  res.success = adjustImpedanceControllerStiffness(req,res);

  return true;
}

bool DemoInterface::graspCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
  if (req.data)
  {
    franka_gripper::GraspGoal grasping_goal;
    grasping_goal.width = 0.0;
    grasping_goal.force = 2.0;
    grasping_goal.speed = 0.01;
    grasping_goal.epsilon.inner = 0.5;
    grasping_goal.epsilon.outer = 0.5;

    if (!gripper_grasp_client_->waitForServer(ros::Duration(1))){
      ROS_ERROR("Cannot reach GraspAction Server");
      res.success = false;
      res.message = "Cannot reach GraspAction Server";
      return false;
    }

    gripper_grasp_client_->sendGoalAndWait(grasping_goal);
    if (gripper_grasp_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_ERROR("Error in the grasp goal: %s", gripper_grasp_client_->getState().getText().c_str());
      res.success = false;
      res.message = gripper_grasp_client_->getState().getText();
      return false;
    }

    res.success = true;
    res.message = gripper_grasp_client_->getState().getText();

    return true;
  } else {
    franka_gripper::MoveGoal move_goal;
    move_goal.width = 0.0762;
    move_goal.speed = 0.01;

    if (!gripper_move_client_->waitForServer(ros::Duration(1))){
      ROS_ERROR("Cannot reach MoveAction Server");
      res.success = false;
      res.message = "Cannot reach MoveAction Server";
      return false;
    }

    gripper_move_client_->sendGoalAndWait(move_goal);
    if (gripper_move_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_ERROR("Error in the grasp goal: %s", gripper_move_client_->getState().getText().c_str());
      res.success = false;
      res.message = gripper_move_client_->getState().getText();
      return false;
    }

    res.success = true;
    res.message = gripper_move_client_->getState().getText();

    return true;
  }
}

bool DemoInterface::userSyncCallback(panda_pbd::UserSyncRequest &req, panda_pbd::UserSyncResponse &res){
  boost::shared_ptr<geometry_msgs::WrenchStamped const> last_external_wrench_ptr;
  bool threshold_exceeded = false;

  adjustImpedanceControllerStiffness(1000, 200, 10);
  ROS_WARN("Setting the robot to be stiff -- to be pushable");

  while (!threshold_exceeded)
  {
    last_external_wrench_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
            "/franka_state_controller/F_ext");
    if (last_external_wrench_ptr != nullptr)
    {
      last_wrench_ = *last_external_wrench_ptr;
      if (std::abs(last_wrench_.wrench.force.x) > req.force_threshold.x ||
          std::abs(last_wrench_.wrench.force.y) > req.force_threshold.y ||
          std::abs(last_wrench_.wrench.force.z) > req.force_threshold.z)
      {
        ROS_INFO("User unlocked the robot");
        ROS_INFO("[%f %f %f] sensed",
                last_wrench_.wrench.force.x,
                last_wrench_.wrench.force.y,
                last_wrench_.wrench.force.z);
        threshold_exceeded = true;
      } else {
        ROS_INFO("Not enough");
        ROS_INFO("[%f %f %f] sensed",
                last_wrench_.wrench.force.x,
                last_wrench_.wrench.force.y,
                last_wrench_.wrench.force.z);
      }
    }
  }
  res.success = true;
  return res.success;
}
