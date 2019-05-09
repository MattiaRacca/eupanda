#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface():
  nh_("~")
{
  // Service servers
  kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
  open_gripper_server_ = nh_.advertiseService("open_gripper", &DemoInterface::openGripperCallback, this);
  close_gripper_server_ = nh_.advertiseService("close_gripper", &DemoInterface::closeGripperCallback, this);

  // TODO: to be removed once we have the pbd implemented
  move_to_ee_test_server_ = nh_.advertiseService("move_to_ee_test", &DemoInterface::moveToEETestCallback, this);

  // Publishers
  equilibrium_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose", 10);

  // Action servers
  move_to_contact_server_ = new actionlib::SimpleActionServer<panda_pbd::MoveToContactAction>(
          nh_, "move_to_contact_server",boost::bind(&DemoInterface::moveToContactCallback, this, _1), false);
  move_to_contact_server_->start();

  move_to_ee_server_ = new actionlib::SimpleActionServer<panda_pbd::MoveToEEAction>(
          nh_, "move_to_ee_server",boost::bind(&DemoInterface::moveToEECallback, this, _1), false);
  move_to_ee_server_->start();

  user_sync_server_ = new actionlib::SimpleActionServer<panda_pbd::UserSyncAction>(
          nh_, "user_sync_server",boost::bind(&DemoInterface::userSyncCallback, this, _1), false);
  user_sync_server_->start();

  // Reconfigure clients
  cartesian_impedance_dynamic_reconfigure_client_ = nh_.
      serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");

  cartesian_impedance_direction_dynamic_reconfigure_client_ = nh_.
          serviceClient<dynamic_reconfigure::Reconfigure>
                  ("/dynamic_reconfigure_cartesian_impedance_direction_param_node/set_parameters");

  // Service clients
  forcetorque_collision_client_ = nh_.
                                  serviceClient<franka_control::SetForceTorqueCollisionBehavior>
                                          ("/franka_control/set_force_torque_collision_behavior");
  controller_manager_switch_ = nh_.
          serviceClient<controller_manager_msgs::SwitchController>
                  ("/controller_manager/switch_controller");

  // Action clients
  gripper_grasp_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", true);
  gripper_move_client_ = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("/franka_gripper/move", true);

  // TODO: to be removed once we have the pbd implemented
  move_to_ee_client_ = new actionlib::SimpleActionClient<panda_pbd::MoveToEEAction>(
          "/demo_interface_node/move_to_ee_server", true);

  gripper_grasp_client_->waitForServer();
  gripper_move_client_->waitForServer();

  // Controller Manager interface
  ROS_DEBUG("Waiting for the controller manager");
  controller_manager_switch_.waitForExistence();

  ROS_DEBUG("Starting the %s...", IMPEDANCE_CONTROLLER.c_str());
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.start_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_controller.request.strictness = 2;

  // TODO: proper handling here please
  bool loaded_controller = false;
  while (!loaded_controller){
    controller_manager_switch_.call(switch_controller);
    loaded_controller = switch_controller.response.ok;
    if (loaded_controller){
      ROS_DEBUG("%s started", IMPEDANCE_CONTROLLER.c_str());
    } else {
      ROS_ERROR("Cannot start %s. Trying again in 2 seconds...", IMPEDANCE_CONTROLLER.c_str());
      ros::Duration(2).sleep();
    }
  }

  ROS_INFO("Demo Interface: initialization completed");
}

geometry_msgs::PoseStamped DemoInterface::getPose(const std::string ref_frame, const std::string child_frame)
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

geometry_msgs::PoseStamped DemoInterface::getEEPose()
{
  return getPose(BASE_FRAME, EE_FRAME);
}


bool DemoInterface::adjustFTThreshold(double ft_multiplier)
{
  franka_control::SetForceTorqueCollisionBehavior collision_srv;

  boost::array<double, 6> force_threshold{ {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} };
  boost::array<double, 7> torque_threshold{ {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} };

  if (ft_multiplier < 1.0){
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
bool DemoInterface::adjustImpedanceControllerStiffness(double transl_stiff = 200.0,
                                                       double rotat_stiff = 10.0,
                                                       double ft_mult = 1.0) {

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

bool DemoInterface::adjustImpedanceControllerStiffness(panda_pbd::EnableTeaching::Request &req,
                                                       panda_pbd::EnableTeaching::Response &res)
{
  bool result;
  switch (req.teaching) {
    case 0: {
      ROS_INFO("Teaching mode deactivated...");
      result = adjustImpedanceControllerStiffness();
      break;
    }
    case 1: {
      ROS_INFO("Full Teaching mode activated...");
      result = adjustImpedanceControllerStiffness(0.0, 0.0, req.ft_threshold_multiplier);
      break;
    }
    case 2: {
      ROS_INFO("Position Teaching mode activated...");
      result = adjustImpedanceControllerStiffness(0.0, 10.0, req.ft_threshold_multiplier);
      break;
    }
    case 3: {
      ROS_INFO("Orientation Teaching mode activated...");
      result = adjustImpedanceControllerStiffness(200.0, 0.0, req.ft_threshold_multiplier);
      break;
    }
    default: {
      ROS_ERROR("Unknown value of teaching - nothing happened");
      result = false;
    }
  }

  res.success = result;
  res.ee_pose = getEEPose();

  return result;
}

bool DemoInterface::adjustDirectionControllerParameters(geometry_msgs::Vector3 direction,
                                                        double speed = 0.0,
                                                        double transl_stiff = 1500.0,
                                                        double rotat_stiff = 300.0,
                                                        double ft_mult = 10.0
){
  dynamic_reconfigure::Reconfigure direction_srv;
  dynamic_reconfigure::DoubleParameter translational_stiff, rotational_stiff;
  dynamic_reconfigure::DoubleParameter vx_d, vy_d, vz_d, speed_d;

  adjustFTThreshold(ft_mult);

  translational_stiff.name = "translational_stiffness";
  rotational_stiff.name = "rotational_stiffness";
  vx_d.name = "vx_d";
  vy_d.name = "vy_d";
  vz_d.name = "vz_d";
  speed_d.name = "speed";

  translational_stiff.value = transl_stiff;
  rotational_stiff.value = rotat_stiff;
  vx_d.value = direction.x;
  vy_d.value = direction.y;
  vz_d.value = direction.z;
  speed_d.value = speed;

  direction_srv.request.config.doubles.push_back(translational_stiff);
  direction_srv.request.config.doubles.push_back(rotational_stiff);
  direction_srv.request.config.doubles.push_back(vx_d);
  direction_srv.request.config.doubles.push_back(vy_d);
  direction_srv.request.config.doubles.push_back(vz_d);
  direction_srv.request.config.doubles.push_back(speed_d);

  ROS_INFO("Changing %s parameters...", IMPEDANCE_DIRECTION_CONTROLLER.c_str());
  return cartesian_impedance_direction_dynamic_reconfigure_client_.call(direction_srv);
}

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
                                                panda_pbd::EnableTeaching::Response &res)
{
  res.success = adjustImpedanceControllerStiffness(req,res);
  return res.success;
}

bool DemoInterface::openGripperCallback(panda_pbd::OpenGripper::Request &req,
                                        panda_pbd::OpenGripper::Response &res) {
  franka_gripper::MoveGoal move_goal;
  // Width in m between the gripper's two fingers
  // TODO: Do we have a way to check the range for the gripper width?

  move_goal.width = std::max(0.0, std::min(req.width, 0.0762));
  move_goal.speed = 0.01; // in m/s

  if (!gripper_move_client_->waitForServer(ros::Duration(1))){
    ROS_ERROR("Cannot reach MoveAction Server");
    res.success = false;
    return res.success;
  }

  gripper_move_client_->sendGoalAndWait(move_goal);
  if (gripper_move_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Error in the grasp goal: %s", gripper_move_client_->getState().getText().c_str());
    res.success = false;
  } else {
    res.success = true;
  }
  return res.success;;
}

bool DemoInterface::closeGripperCallback(panda_pbd::CloseGripper::Request &req,
                                         panda_pbd::CloseGripper::Response &res) {
  franka_gripper::GraspGoal grasping_goal;

  // TODO: Do we have a way to check the range for the gripper width?
  // TODO: Do we want to enforce a force range?

  grasping_goal.width = std::max(0.0, std::min(req.width, 0.0762));;
  grasping_goal.force = req.force;
  grasping_goal.speed = 0.01; // in m/s

  // TODO: this epsilon value will never trigger an error basically (neglectful handling)
  grasping_goal.epsilon.inner = 0.5;
  grasping_goal.epsilon.outer = 0.5;

  if (!gripper_grasp_client_->waitForServer(ros::Duration(1))){
    ROS_ERROR("Cannot reach GraspAction Server");
    res.success = false;
    return res.success;
  }

  gripper_grasp_client_->sendGoalAndWait(grasping_goal);
  if (gripper_grasp_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Error in the grasp goal: %s", gripper_grasp_client_->getState().getText().c_str());
    res.success = false;
  } else {
    res.success = true;
  }
  return res.success;
}

void DemoInterface::userSyncCallback(const panda_pbd::UserSyncGoalConstPtr &goal){
  boost::shared_ptr<geometry_msgs::WrenchStamped const> last_external_wrench_ptr;
  user_sync_result_.unlock = false;

  adjustImpedanceControllerStiffness(1000.0, 200.0, 10.0);
  ROS_WARN("Setting the robot to be stiff (to be pushable)");

  double force_threshold = goal.get()->force_threshold;

  while (!user_sync_result_.unlock)
  {
    /* The external wrench is in the EE frame
     * Positive value when the force is applied AGAINST the axis
     */
    last_external_wrench_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
            "/franka_state_controller/F_ext");

    if (last_external_wrench_ptr != nullptr)
    {
      last_wrench_ = *last_external_wrench_ptr;
      if (std::abs(last_wrench_.wrench.force.x) > force_threshold ||
          std::abs(last_wrench_.wrench.force.y) > force_threshold ||
          std::abs(last_wrench_.wrench.force.z) > force_threshold)
      {
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
  user_sync_server_->setSucceeded(user_sync_result_);

  // setting stiffnes back to default
  adjustImpedanceControllerStiffness();
}

void DemoInterface::moveToContactCallback(const panda_pbd::MoveToContactGoalConstPtr &goal){
  // TODO: do proper error handling
  ROS_DEBUG("Received MoveToContact request");

  ROS_INFO("Trying to switch to %s...", IMPEDANCE_DIRECTION_CONTROLLER.c_str());
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.stop_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_controller.request.start_controllers.push_back(IMPEDANCE_DIRECTION_CONTROLLER);
  switch_controller.request.strictness = 2;

  controller_manager_switch_.call(switch_controller);
  ROS_INFO("and... %s", switch_controller.response.ok ? "SUCCESS" : "FAILURE");
  if (!switch_controller.response.ok)
    return;

  ROS_WARN("Setting the robot to be stiff (to be able to push against, if needed)");
  adjustDirectionControllerParameters(goal.get()->direction, goal.get()->speed);

  ROS_DEBUG("%s until contact...", IMPEDANCE_DIRECTION_CONTROLLER.c_str());

  bool in_contact = false;
  while(!in_contact)
  {
    auto last_external_wrench_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
            "/franka_state_controller/F_ext");

    if (last_external_wrench_ptr != nullptr)
    {
      last_wrench_ = *last_external_wrench_ptr;
      if (std::abs(last_wrench_.wrench.force.x) > goal->force_threshold ||
          std::abs(last_wrench_.wrench.force.y) > goal->force_threshold ||
          std::abs(last_wrench_.wrench.force.z) > goal->force_threshold)
      {
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
        move_to_contact_server_->publishFeedback(move_to_contact_feedback_);
      }
    }
  }

  move_to_contact_result_.ee_pose = getEEPose();
  move_to_contact_server_->setSucceeded(move_to_contact_result_);

  ROS_INFO("Trying to switch back to %s...", IMPEDANCE_CONTROLLER.c_str());
  controller_manager_msgs::SwitchController switch_back_controller;
  switch_back_controller.request.start_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_back_controller.request.stop_controllers.push_back(IMPEDANCE_DIRECTION_CONTROLLER);
  switch_back_controller.request.strictness = 2;

  adjustImpedanceControllerStiffness();
  controller_manager_switch_.call(switch_back_controller);
  ROS_INFO("and... %s", switch_back_controller.response.ok ? "SUCCESS" : "FAILURE");
  if (!switch_back_controller.response.ok)
    return;
}

void DemoInterface::moveToEECallback(const panda_pbd::MoveToEEGoalConstPtr &goal){
  ROS_DEBUG("Received MoveToEE request");
  // TODO: assuming the cartesian impedance controller is active

  ROS_WARN("Setting the robot to be stiff (to execute trajectory)");
  adjustImpedanceControllerStiffness(1500.0, 300.0, 1.0);

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

  double position_speed_target = goal->position_speed; // m/s
  double rotation_speed_target = goal->rotation_speed; // rad/s

  // Compute "pace" of motion
  Eigen::Vector3d position_difference = target_position - current_position;
  Eigen::Quaterniond rotation_difference(target_orientation * current_orientation.inverse());

  // Convert to axis angle
  Eigen::AngleAxisd rotation_difference_angle_axis(rotation_difference);

  double desired_time = std::max(rotation_difference_angle_axis.angle()/rotation_speed_target,
                           position_difference.norm()/position_speed_target);
  double progression = 0.0;
  bool motion_done = false;

  ros::Time starting_time = ros::Time::now();

  while(!motion_done)
  {
    ros::Time current_time = ros::Time::now();
    progression = (current_time - starting_time).toSec();

    double tau = std::min(progression/desired_time, 1.0);
    // Position
    Eigen::Vector3d desired_position = current_position*(1 - tau) + target_position*tau;
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

    equilibrium_pose_publisher_.publish(desired_pose);

    if(tau < 1.0){
      move_to_ee_feedback_.progression = tau;
      // TODO: way to make this throttled?
      move_to_ee_server_->publishFeedback(move_to_ee_feedback_);
    } else {
      motion_done = true;
    }
  }

  // TODO: remove this after testing
  ros::Duration(1).sleep();
  move_to_ee_result_.final_pose = getEEPose();
  move_to_ee_server_->setSucceeded(move_to_ee_result_);

  ROS_WARN("Setting the robot to default impedance controller");
  adjustImpedanceControllerStiffness();
}

bool DemoInterface::moveToEETestCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res){
  ROS_INFO("Test for the move to EE... going in teaching mode");
  // TODO: error handling
  // TODO: assuming cartesian impedance controller here
  auto result = adjustImpedanceControllerStiffness(0.0, 0.0, 5.0);
  ROS_INFO("Move the robot to new position");
  ros::Duration(20).sleep();
  geometry_msgs::PoseStamped goal_pose = getEEPose();
  ROS_INFO("Move it back now!");
  ros::Duration(20).sleep();

  geometry_msgs::PoseStamped current_pose = getEEPose();
  ROS_WARN("Move to EE test: current position [%f, %f, %f] and orientation [%f %f %f %f]",
          current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
          current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);

  if (!move_to_ee_client_->waitForServer(ros::Duration(1))){
    ROS_ERROR("Cannot reach MoveToEE Server");
    res.success = false;
    return res.success;
  }

  panda_pbd::MoveToEEGoal goal;
  goal.pose = goal_pose;
  goal.position_speed = 0.04;
  goal.rotation_speed = 1.0;

  ROS_INFO("Sending the goal, to itself...");

  move_to_ee_client_->sendGoalAndWait(goal);
  if (move_to_ee_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Error in the moveToEE action: %s", move_to_ee_client_->getState().getText().c_str());
    res.success = false;
  } else {
    res.success = true;
  }

  current_pose = getEEPose();
  ROS_WARN("Move to EE test: current position [%f, %f, %f] and orientation [%f %f %f %f]",
           current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
           current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);

  ROS_WARN("Was trying to move here: goal position [%f, %f, %f] and orientation [%f %f %f %f]",
           goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
           goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);

  return res.success;
}