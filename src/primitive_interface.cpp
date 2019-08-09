#include "panda_pbd/primitive_interface.h"

PrimitiveInterface::PrimitiveInterface():
  nh_("~")
{
  // Service servers
  kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &PrimitiveInterface::kinestheticTeachingCallback, this);
  open_gripper_server_ = nh_.advertiseService("open_gripper", &PrimitiveInterface::openGripperCallback, this);
  close_gripper_server_ = nh_.advertiseService("close_gripper", &PrimitiveInterface::closeGripperCallback, this);
  move_fingers_server_ = nh_.advertiseService("move_fingers", &PrimitiveInterface::moveFingersCallback, this);
  apply_force_fingers_server_ = nh_.advertiseService("apply_force_fingers",
          &PrimitiveInterface::applyForceFingersCallback, this);

  // Publishers
  equilibrium_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose", 10);
  interface_state_publisher_ = nh_.advertise<std_msgs::Int32>("interface_state",10);

  // Subscribers
  franka_state_subscriber_ = nh_.subscribe("/franka_state_controller/franka_states", 10,
          &PrimitiveInterface::frankaStateCallback, this);

  interface_state_.store(1); // TODO: not sure this is right

  // Action servers
  move_to_contact_server_ = new actionlib::SimpleActionServer<panda_pbd::MoveToContactAction>(
    nh_, "move_to_contact_server", boost::bind(&PrimitiveInterface::moveToContactCallback, this, _1), false);
  move_to_contact_server_->start();

  move_to_ee_server_ = new actionlib::SimpleActionServer<panda_pbd::MoveToEEAction>(
    nh_, "move_to_ee_server", boost::bind(&PrimitiveInterface::moveToEECallback, this, _1), false);
  move_to_ee_server_->start();

  user_sync_server_ = new actionlib::SimpleActionServer<panda_pbd::UserSyncAction>(
    nh_, "user_sync_server", boost::bind(&PrimitiveInterface::userSyncCallback, this, _1), false);
  user_sync_server_->start();

  // Reconfigure clients
  cartesian_impedance_dynamic_reconfigure_client_ = nh_.
      serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");

  // Service clients
  forcetorque_collision_client_ = nh_.
                                  serviceClient<franka_control::SetForceTorqueCollisionBehavior>
                                  ("/franka_control/set_force_torque_collision_behavior");
  controller_manager_switch_ = nh_.
                               serviceClient<controller_manager_msgs::SwitchController>
                               ("/controller_manager/switch_controller");

  controller_manager_list_ = nh_.
                             serviceClient<controller_manager_msgs::ListControllers>
                             ("/controller_manager/list_controllers");

  // Action clients
  gripper_grasp_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", true);
  gripper_move_client_ = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("/franka_gripper/move", true);

  // TODO: to be removed once we have the pbd implemented
  move_to_ee_client_ = new actionlib::SimpleActionClient<panda_pbd::MoveToEEAction>(
    "/primitive_interface_node/move_to_ee_server", true);
  move_to_contact_client_ = new actionlib::SimpleActionClient<panda_pbd::MoveToContactAction>(
    "/primitive_interface_node/move_to_contact_server", true);

  gripper_grasp_client_->waitForServer();
  gripper_move_client_->waitForServer();

  // Controller Manager interface
  ROS_DEBUG("Waiting for the controller manager");
  bool manager_is_there = controller_manager_switch_.waitForExistence(ros::Duration(10));

  if (!manager_is_there)
  {
    ROS_ERROR("Controller Manager not reachable... Expect errors");
  }

  bool cartesian_impedance_found = false;
  controller_manager_msgs::ListControllers list;
  if (controller_manager_list_.call(list))
  {
    for (const controller_manager_msgs::ControllerState &controller : list.response.controller)
    {
      ROS_DEBUG("Controller %s found (state: %s)", controller.name.c_str(), controller.state.c_str());
      if (controller.name == IMPEDANCE_CONTROLLER)
      {
        cartesian_impedance_found = true;
      }
    }
  }

  if (!cartesian_impedance_found)
  {
    ROS_ERROR("Cartesian Impedance controller not found... Expect errors");
  }

  ROS_DEBUG("Starting the %s...", IMPEDANCE_CONTROLLER.c_str());
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.start_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_controller.request.strictness = 2;

  bool loaded_controller = false;
  while (!loaded_controller)
  {
    controller_manager_switch_.call(switch_controller);
    loaded_controller = switch_controller.response.ok;
    if (loaded_controller)
    {
      ROS_DEBUG("%s started", IMPEDANCE_CONTROLLER.c_str());
    }
    else
    {
      ROS_ERROR("Cannot start %s. Trying again in 2 seconds...", IMPEDANCE_CONTROLLER.c_str());
      ros::Duration(2).sleep();
    }
  }

  ROS_INFO("======= Primitive Interface: initialization completed =======");
}

bool PrimitiveInterface::isInterfaceReady()
{
  int state = interface_state_.load();
  if(state != 1){
    ROS_ERROR("Robot is not ready...");
    if(state == 0)
      ROS_ERROR("Robot is in an error state - please recover from error first!");
    if(state == 2)
      ROS_ERROR("Interface is busy - wait for the other requests to be processed!");
    return false;
  } else {
    return true;
  }
}

geometry_msgs::PoseStamped PrimitiveInterface::getPose(const std::string ref_frame, const std::string child_frame)
{
  std::string tf_err_msg;
  tf::StampedTransform transform;

  if (!pose_listener_.waitForTransform(ref_frame, child_frame, ros::Time(0),
                                       ros::Duration(0.5), ros::Duration(0.01),
                                       &tf_err_msg))
  {
    ROS_ERROR_STREAM("Unable to get pose from TF: " << tf_err_msg);
  }
  else
  {
    try
    {
      pose_listener_.lookupTransform(ref_frame, child_frame,
                                     ros::Time(0), // get latest available
                                     transform);
    }
    catch (const tf::TransformException &e)
    {
      ROS_ERROR_STREAM("Error in lookupTransform of " << child_frame << " in "
                       << ref_frame);
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

geometry_msgs::PoseStamped PrimitiveInterface::getEEPose()
{
  return getPose(BASE_FRAME, EE_FRAME);
}

bool PrimitiveInterface::adjustFTThreshold(double ft_multiplier)
{
  franka_control::SetForceTorqueCollisionBehavior collision_srv;

  boost::array<double, 6> force_threshold{ {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} };
  boost::array<double, 7> torque_threshold{ {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} };

  if (ft_multiplier < 1.0)
  {
    ROS_WARN("ForceTorque Multiplier has to be greater than 1.0...");
    ROS_WARN("Setting to Default (1.0)");
    ft_multiplier = 1.0;
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

bool PrimitiveInterface::adjustImpedanceControllerStiffness(geometry_msgs::PoseStamped desired_pose,
        double transl_stiff = 200.0, double rotat_stiff = 10.0, double ft_mult = 1.0)
{
  equilibrium_pose_publisher_.publish(desired_pose);
  ros::Duration(0.5).sleep(); // to allow the controller to receive the new equilibrium pose

  dynamic_reconfigure::Reconfigure stiffness_srv;

  dynamic_reconfigure::DoubleParameter translational_stiff, rotational_stiff;
  translational_stiff.name = "translational_stiffness";
  rotational_stiff.name = "rotational_stiffness";

  adjustFTThreshold(ft_mult);
  translational_stiff.value = transl_stiff;
  rotational_stiff.value = rotat_stiff;

  stiffness_srv.request.config.doubles.push_back(translational_stiff);
  stiffness_srv.request.config.doubles.push_back(rotational_stiff);

  ROS_INFO("Changing %s parameters...", IMPEDANCE_CONTROLLER.c_str());
  return cartesian_impedance_dynamic_reconfigure_client_.call(stiffness_srv);
}

bool PrimitiveInterface::adjustImpedanceControllerStiffness(double transl_stiff = 200.0,
    double rotat_stiff = 10.0,
    double ft_mult = 1.0)
{

  auto ee_pose = getEEPose();
  equilibrium_pose_publisher_.publish(ee_pose);
  ros::Duration(0.5).sleep(); // to allow the controller to receive the new equilibrium pose

  dynamic_reconfigure::Reconfigure stiffness_srv;

  dynamic_reconfigure::DoubleParameter translational_stiff, rotational_stiff;
  translational_stiff.name = "translational_stiffness";
  rotational_stiff.name = "rotational_stiffness";

  adjustFTThreshold(ft_mult);
  translational_stiff.value = transl_stiff;
  rotational_stiff.value = rotat_stiff;

  stiffness_srv.request.config.doubles.push_back(translational_stiff);
  stiffness_srv.request.config.doubles.push_back(rotational_stiff);

  ROS_INFO("Changing %s parameters...", IMPEDANCE_CONTROLLER.c_str());
  return cartesian_impedance_dynamic_reconfigure_client_.call(stiffness_srv);
}

bool PrimitiveInterface::adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req,
    panda_pbd::EnableTeaching::Response &res)
{
  bool result;
  switch (req.teaching)
  {
  case 0:
  {
    ROS_INFO("Teaching mode deactivated...");
    result = adjustImpedanceControllerStiffness();
    break;
  }
  case 1:
  {
    ROS_INFO("Full Teaching mode activated...");
    result = adjustImpedanceControllerStiffness(0.0, 0.0, req.ft_threshold_multiplier);
    break;
  }
  case 2:
  {
    ROS_INFO("Position Teaching mode activated...");
    result = adjustImpedanceControllerStiffness(0.0, 10.0, req.ft_threshold_multiplier);
    break;
  }
  case 3:
  {
    ROS_INFO("Orientation Teaching mode activated...");
    result = adjustImpedanceControllerStiffness(200.0, 0.0, req.ft_threshold_multiplier);
    break;
  }
  default:
  {
    ROS_ERROR("Unknown value of teaching - nothing happened");
    result = false;
  }
  }

  res.success = result;
  res.ee_pose = getEEPose();

  return result;
}

bool PrimitiveInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
    panda_pbd::EnableTeaching::Response &res)
{
  if(isInterfaceReady()){
    interface_state_.store(2);
    res.success = adjustImpedanceControllerStiffness(req, res);
    interface_state_.store(1);
    return res.success;
  } else {
    res.success = false;
    return res.success;
  }
}

bool PrimitiveInterface::moveFingersCallback(panda_pbd::MoveFingers::Request &req,
                                             panda_pbd::MoveFingers::Response &res){
  franka_gripper::MoveGoal move_goal;
  move_goal.width = std::max(0.0, std::min(req.width, 0.08));
  // TODO: make this parameter a rosparam?
  move_goal.speed = 0.03;// in m/s

  if (!gripper_move_client_->waitForServer(ros::Duration(1)))
  {
    ROS_ERROR("Cannot reach MoveAction Server");
    res.success = false;
    return res.success;
  }

  if(isInterfaceReady()){
    interface_state_.store(2);
    gripper_move_client_->sendGoalAndWait(move_goal);
    if (gripper_move_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("Move fingers primitive failed: %s", gripper_move_client_->getState().getText().c_str());
      res.success = false;
    }
    else
    {
      res.success = true;
    }
    interface_state_.store(1);
    return res.success;
  } else {
    res.success = false;
    return res.success;
  }

}

bool PrimitiveInterface::applyForceFingersCallback(panda_pbd::ApplyForceFingers::Request &req,
                                                   panda_pbd::ApplyForceFingers::Response &res){
  franka_gripper::GraspGoal grasping_goal;

  // range of forces is from 20 N to 100 N
  grasping_goal.width = 0.0;
  grasping_goal.force = req.force;
  grasping_goal.speed = 0.03; // in m/s

  // this epsilon value will never trigger an error basically (neglectful handling)
  grasping_goal.epsilon.inner = 0.5;
  grasping_goal.epsilon.outer = 0.5;

  if (!gripper_grasp_client_->waitForServer(ros::Duration(1)))
  {
    ROS_ERROR("Cannot reach GraspAction Server");
    res.success = false;
    return res.success;
  }

  if(isInterfaceReady()) {
    interface_state_.store(2);
    gripper_grasp_client_->sendGoalAndWait(grasping_goal);
    if (gripper_grasp_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR("Apply Force with Fingers primitived failed: %s", gripper_grasp_client_->getState().getText().c_str());
      res.success = false;
    } else {
      res.success = true;
    }
    interface_state_.store(1);
    return res.success;
  } else {
    res.success = false;
    return res.success;
  }
}

bool PrimitiveInterface::openGripperCallback(panda_pbd::OpenGripper::Request &req,
    panda_pbd::OpenGripper::Response &res)
{
  franka_gripper::MoveGoal move_goal;
  move_goal.width = std::max(0.0, std::min(req.width, 0.08));
  move_goal.speed = 0.01; // in m/s

  if (!gripper_move_client_->waitForServer(ros::Duration(1)))
  {
    ROS_ERROR("Cannot reach MoveAction Server");
    res.success = false;
    return res.success;
  }

  if(isInterfaceReady()) {
    interface_state_.store(2);
    gripper_move_client_->sendGoalAndWait(move_goal);
    if (gripper_move_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR("Error in the grasp goal: %s", gripper_move_client_->getState().getText().c_str());
      res.success = false;
    } else {
      res.success = true;
    }
    interface_state_.store(1);
    return res.success;
  } else {
    res.success = false;
    return res.success;
  }
}

bool PrimitiveInterface::closeGripperCallback(panda_pbd::CloseGripper::Request &req,
    panda_pbd::CloseGripper::Response &res)
{
  franka_gripper::GraspGoal grasping_goal;

  // TODO: Do we want to enforce a force range?
  grasping_goal.width = std::max(0.0, std::min(req.width, 0.08));
  grasping_goal.force = req.force;
  grasping_goal.speed = 0.01; // in m/s

  // this epsilon value will never trigger an error basically (neglectful handling)
  grasping_goal.epsilon.inner = 0.5;
  grasping_goal.epsilon.outer = 0.5;

  if (!gripper_grasp_client_->waitForServer(ros::Duration(1)))
  {
    ROS_ERROR("Cannot reach GraspAction Server");
    res.success = false;
    return res.success;
  }

  if(isInterfaceReady()) {
    interface_state_.store(2);
    gripper_grasp_client_->sendGoalAndWait(grasping_goal);
    if (gripper_grasp_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR("Error in the grasp goal: %s", gripper_grasp_client_->getState().getText().c_str());
      res.success = false;
    } else {
      res.success = true;
    }
    interface_state_.store(1);
    return res.success;
  } else {
    res.success = false;
    return res.success;
  }
}

void PrimitiveInterface::userSyncCallback(const panda_pbd::UserSyncGoalConstPtr &goal)
{
  boost::shared_ptr<geometry_msgs::WrenchStamped const> last_external_wrench_ptr;
  user_sync_result_.unlock = false;
  bool error_happened = false;

  adjustImpedanceControllerStiffness(1000.0, 200.0, 10.0);
  ROS_WARN("Setting the robot to be stiff (to be pushable)");

  if(isInterfaceReady()) {
    interface_state_.store(2);
    double force_threshold = goal.get()->force_threshold;

    while (!user_sync_result_.unlock) {
      if (interface_state_.load() == 0) {
        ROS_ERROR("Robot is in an error state - please recover from error first!");
        user_sync_result_.unlock = false;
        error_happened = true;
        break;
      }

      /* The external wrench is in the EE frame
       * Positive value when the force is applied AGAINST the axis
       */
      last_external_wrench_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
              "/franka_state_controller/F_ext");

      if (last_external_wrench_ptr != nullptr) {
        last_wrench_ = *last_external_wrench_ptr;
        if (std::abs(last_wrench_.wrench.force.x) > force_threshold ||
            std::abs(last_wrench_.wrench.force.y) > force_threshold ||
            std::abs(last_wrench_.wrench.force.z) > force_threshold) {
          ROS_DEBUG("Robot unlocked!");
          ROS_DEBUG("[%f %f %f] sensed",
                    last_wrench_.wrench.force.x,
                    last_wrench_.wrench.force.y,
                    last_wrench_.wrench.force.z);
          user_sync_result_.unlock = true;
        } else {
          ROS_DEBUG_THROTTLE(10, "Not enough force ([%f %f %f] sensed)",
                             last_wrench_.wrench.force.x,
                             last_wrench_.wrench.force.y,
                             last_wrench_.wrench.force.z);
          user_sync_feedback_.contact_forces.x = last_wrench_.wrench.force.x;
          user_sync_feedback_.contact_forces.y = last_wrench_.wrench.force.y;
          user_sync_feedback_.contact_forces.z = last_wrench_.wrench.force.z;
          user_sync_server_->publishFeedback(user_sync_feedback_);
        }
      }
    }
    if(!error_happened)
      interface_state_.store(1);
  } else {
    error_happened = true;
  }
  if (error_happened) {
    user_sync_server_->setAborted(user_sync_result_);
  } else {
    user_sync_server_->setSucceeded(user_sync_result_);
  }
  // setting stiffness back to default
  adjustImpedanceControllerStiffness();
}

void PrimitiveInterface::moveToContactCallback(const panda_pbd::MoveToContactGoalConstPtr &goal) {
  ROS_DEBUG("Received MoveToContact request");
  ROS_WARN("Setting the robot to be stiff (to execute trajectory)");
  adjustImpedanceControllerStiffness(1500.0, 300.0, 10.0);

  // Put current pose in Eigen form
  auto current_pose = getEEPose();
  Eigen::Vector3d current_position;
  current_position << current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z;
  Eigen::Quaterniond current_orientation;
  current_orientation.coeffs() << current_pose.pose.orientation.x, current_pose.pose.orientation.y,
          current_pose.pose.orientation.z, current_pose.pose.orientation.w;

  // Put target pose in Eigen form
  Eigen::Vector3d target_position;
  target_position << goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z;
  Eigen::Quaterniond target_orientation;

  target_orientation.coeffs() << goal->pose.pose.orientation.x, goal->pose.pose.orientation.y,
          goal->pose.pose.orientation.z, goal->pose.pose.orientation.w;

  if (current_orientation.coeffs().dot(target_orientation.coeffs()) < 0.0) {
    target_orientation.coeffs() << -target_orientation.coeffs();
  }

  double expected_time = 0.0;

  if (goal->rotation_speed < 0.0){
    // The rotation speed is computed starting from the position speed
    ROS_DEBUG("MoveToContact primitive: rotation speed not specified - Computing it with respect to the position speed");

    double position_speed_target = goal->position_speed; // m/s

    // Compute "pace" of motion
    Eigen::Vector3d position_difference = target_position - current_position;
    Eigen::Quaterniond rotation_difference(target_orientation * current_orientation.inverse());

    // Convert to axis angle
    Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

    double time_position = position_difference.norm() / position_speed_target;

    double rotation_speed_target = std::min(max_rotation_speed,
                                            position_speed_target/position_difference.norm()*rotation_difference_angle_axis.angle());

    double time_orientation = rotation_speed_target / rotation_difference_angle_axis.angle();
    expected_time = std::max(time_orientation, time_position);

    if (expected_time == time_orientation) {
      ROS_WARN("MovetoContact primitive: position speed was capped to respect max_rotation_speed");
      ROS_WARN("Angle of rotation is %f, rotation_speed_target is %f - required time is %f",
               rotation_difference_angle_axis.angle(), rotation_speed_target, time_orientation);
      ROS_WARN("Position delta is %f, position_speed_target is %f - required time is %f",
               position_difference.norm(), position_speed_target, time_position);
    }
  } else {
    double position_speed_target = goal->position_speed; // m/s
    double rotation_speed_target = goal->rotation_speed; // rad/s

    // Compute "pace" of motion
    Eigen::Vector3d position_difference = target_position - current_position;
    Eigen::Quaterniond rotation_difference(target_orientation * current_orientation.inverse());

    // Convert to axis angle
    Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

    expected_time = std::max(rotation_difference_angle_axis.angle() / rotation_speed_target,
                                    position_difference.norm() / position_speed_target);
  }

  double progression = 0.0;
  bool in_contact = false;
  bool error_happened = false;

  ros::Time starting_time = ros::Time::now();

  if(isInterfaceReady()) {
    interface_state_.store(2);
    while (!in_contact) {
      if (interface_state_.load() == 0) {
        ROS_ERROR("Robot went in an error state while executing move_to_contact - please recover from error first!");
        error_happened = true;
        break;
      }

      ros::Time current_time = ros::Time::now();
      progression = (current_time - starting_time).toSec();

      double tau = progression / expected_time;
      // Position
      Eigen::Vector3d desired_position = current_position * (1 - tau) + target_position * tau;
      // Orientation
      Eigen::Quaterniond desired_orientation = current_orientation.slerp(tau, target_orientation);

      // command to Impedance controller
      geometry_msgs::PoseStamped desired_pose;

      desired_pose.pose.position.x = desired_position[0];
      desired_pose.pose.position.y = desired_position[1];
      desired_pose.pose.position.z = desired_position[2];

      desired_pose.pose.orientation.x = desired_orientation.x();
      desired_pose.pose.orientation.y = desired_orientation.y();
      desired_pose.pose.orientation.z = desired_orientation.z();
      desired_pose.pose.orientation.w = desired_orientation.w();

      // TODO: enforce working space here? and throw error if contact not reached within?
      equilibrium_pose_publisher_.publish(desired_pose);

      auto last_external_wrench_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
              "/franka_state_controller/F_ext");

      if (last_external_wrench_ptr != nullptr) {
        last_wrench_ = *last_external_wrench_ptr;
        if (std::abs(last_wrench_.wrench.force.x) > goal->force_threshold ||
            std::abs(last_wrench_.wrench.force.y) > goal->force_threshold ||
            std::abs(last_wrench_.wrench.force.z) > goal->force_threshold ||
            std::abs(last_wrench_.wrench.torque.x) > goal->torque_threshold ||
            std::abs(last_wrench_.wrench.torque.y) > goal->torque_threshold ||
            std::abs(last_wrench_.wrench.torque.z) > goal->torque_threshold) {
          ROS_DEBUG("Robot touched something!");
          ROS_DEBUG("[%f %f %f] sensed",
                    last_wrench_.wrench.force.x,
                    last_wrench_.wrench.force.y,
                    last_wrench_.wrench.force.z);
          in_contact = true;
        } else {
          ROS_DEBUG_THROTTLE(10, "Not enough force ([%f %f %f] sensed)",
                             last_wrench_.wrench.force.x,
                             last_wrench_.wrench.force.y,
                             last_wrench_.wrench.force.z);
          move_to_contact_feedback_.contact_forces.x = last_wrench_.wrench.force.x;
          move_to_contact_feedback_.contact_forces.y = last_wrench_.wrench.force.y;
          move_to_contact_feedback_.contact_forces.z = last_wrench_.wrench.force.z;
          move_to_contact_feedback_.contact_torques.x = last_wrench_.wrench.torque.x;
          move_to_contact_feedback_.contact_torques.y = last_wrench_.wrench.torque.y;
          move_to_contact_feedback_.contact_torques.z = last_wrench_.wrench.torque.z;
          move_to_contact_server_->publishFeedback(move_to_contact_feedback_);
        }
      }
    }
    if(!error_happened)
      interface_state_.store(1);
  } else {
    error_happened = true;
  }

  // TODO: is this enough to give the EE_pose time to update?
  ros::Duration(1.0).sleep();
  move_to_contact_result_.final_pose = getEEPose();
  move_to_contact_result_.contact_forces.x = last_wrench_.wrench.force.x;
  move_to_contact_result_.contact_forces.y = last_wrench_.wrench.force.y;
  move_to_contact_result_.contact_forces.z = last_wrench_.wrench.force.z;
  move_to_contact_result_.contact_torques.x = last_wrench_.wrench.torque.x;
  move_to_contact_result_.contact_torques.y = last_wrench_.wrench.torque.y;
  move_to_contact_result_.contact_torques.z = last_wrench_.wrench.torque.z;

  if (error_happened){
    move_to_contact_server_->setAborted(move_to_contact_result_);
  } else {
    move_to_contact_server_->setSucceeded(move_to_contact_result_);
  }

  ros::Duration(1.0).sleep(); // to allow the impedance controller to complete the motion
  ROS_WARN("Setting the robot to default Impedance controller");
  adjustImpedanceControllerStiffness();
}

void PrimitiveInterface::moveToEECallback(const panda_pbd::MoveToEEGoalConstPtr &goal)
{
  ROS_DEBUG("Received MoveToEE request");

  ROS_WARN("Setting the robot to be stiff (to execute trajectory)");
  adjustImpedanceControllerStiffness(3000.0, 600.0, 1.0);

  // Put current pose in Eigen form
  auto current_pose = getEEPose();
  Eigen::Vector3d current_position;
  current_position << current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z;
  Eigen::Quaterniond current_orientation;
  current_orientation.coeffs() << current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                             current_pose.pose.orientation.z, current_pose.pose.orientation.w;

  // Put target pose in Eigen form
  Eigen::Vector3d target_position;
  target_position << goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z;
  Eigen::Quaterniond target_orientation;

  target_orientation.coeffs() << goal->pose.pose.orientation.x, goal->pose.pose.orientation.y,
                            goal->pose.pose.orientation.z, goal->pose.pose.orientation.w;

  if (current_orientation.coeffs().dot(target_orientation.coeffs()) < 0.0)
  {
    target_orientation.coeffs() << -target_orientation.coeffs();
  }

  double desired_time = 0.0;

  if(goal->rotation_speed < 0.0){
    // The rotation speed is computed starting from the position speed
    ROS_DEBUG("MoveToEE primitive: rotation speed not specified - Computing it with respect to the position speed");

    double position_speed_target = goal->position_speed; // m/s

    // Compute "pace" of motion
    Eigen::Vector3d position_difference = target_position - current_position;
    Eigen::Quaterniond rotation_difference(target_orientation * current_orientation.inverse());

    // Convert to axis angle
    Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

    double time_position = position_difference.norm() / position_speed_target;

    double rotation_speed_target = std::min(max_rotation_speed,
            position_speed_target/position_difference.norm()*rotation_difference_angle_axis.angle());

    double time_orientation = rotation_speed_target / rotation_difference_angle_axis.angle();
    desired_time = std::max(time_orientation, time_position);

    if (desired_time == time_orientation) {
      ROS_WARN("MovetoEE primitive: position speed was capped to respect max_rotation_speed");
      ROS_WARN("Angle of rotation is %f, rotation_speed_target is %f - required time is %f",
               rotation_difference_angle_axis.angle(), rotation_speed_target, time_orientation);
      ROS_WARN("Position delta is %f, position_speed_target is %f - required time is %f",
               position_difference.norm(), position_speed_target, time_position);
    }

  } else {
    double position_speed_target = goal->position_speed; // m/s
    double rotation_speed_target = goal->rotation_speed; // rad/s

    // Compute "pace" of motion
    Eigen::Vector3d position_difference = target_position - current_position;
    Eigen::Quaterniond rotation_difference(target_orientation * current_orientation.inverse());

    // Convert to axis angle
    Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

    desired_time = std::max(rotation_difference_angle_axis.angle() / rotation_speed_target,
                                   position_difference.norm() / position_speed_target);
  }

  double progression = 0.0;
  bool motion_done = false;
  bool error_happened = false;

  ros::Time starting_time = ros::Time::now();

  // command to Impedance controller
  geometry_msgs::PoseStamped desired_pose;

  if(isInterfaceReady()) {
    interface_state_.store(2);
    while (!motion_done) {
      if (interface_state_.load() == 0) {
        ROS_ERROR("Robot went in an error state while executing move_to_ee - please recover from error first!");
        error_happened = true;
        break;
      }

      ros::Time current_time = ros::Time::now();
      progression = (current_time - starting_time).toSec();

      double tau = std::min(progression / desired_time, 1.0);
      // Position
      Eigen::Vector3d desired_position = current_position * (1 - tau) + target_position * tau;
      // Orientation
      Eigen::Quaterniond desired_orientation = current_orientation.slerp(tau, target_orientation);

      desired_pose.pose.position.x = desired_position[0];
      desired_pose.pose.position.y = desired_position[1];
      desired_pose.pose.position.z = desired_position[2];

      desired_pose.pose.orientation.x = desired_orientation.x();
      desired_pose.pose.orientation.y = desired_orientation.y();
      desired_pose.pose.orientation.z = desired_orientation.z();
      desired_pose.pose.orientation.w = desired_orientation.w();

      equilibrium_pose_publisher_.publish(desired_pose);

      if (tau < 1.0) {
        move_to_ee_feedback_.progression = tau;
        move_to_ee_server_->publishFeedback(move_to_ee_feedback_);
      } else {
        motion_done = true;
      }
    }
    if(!error_happened)
      interface_state_.store(1);
  } else {
    error_happened = true;
  }

  move_to_ee_result_.final_pose = desired_pose;

  if (error_happened){
    move_to_ee_server_->setAborted(move_to_ee_result_);
  } else {
    move_to_ee_server_->setSucceeded(move_to_ee_result_);
  }

  ros::Duration(1.0).sleep(); // to allow the impedance controller to complete the motion
  ROS_WARN("Setting the robot to default Impedance controller");
  adjustImpedanceControllerStiffness();
}

void PrimitiveInterface::frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg){
  if (msg->robot_mode == 4 || msg->robot_mode == 5 || msg->robot_mode == 1){
    // if the robot is REFLEX, USER_STOPPED or IDLE --> stop the interface
    interface_state_.store(0);
  }
  if ((msg->robot_mode != 4 && msg->robot_mode != 5 && msg->robot_mode != 1) && interface_state_.load() == 0){
    interface_state_.store(1);
    adjustImpedanceControllerStiffness();
  }
  std_msgs::Int32 relayed_msg;
  relayed_msg.data = interface_state_.load();
  interface_state_publisher_.publish(relayed_msg);
}
