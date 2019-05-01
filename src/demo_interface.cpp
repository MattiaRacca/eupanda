#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface():
  nh_("~")
{
  // Servers
  kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
  grasp_server_ = nh_.advertiseService("grasp", &DemoInterface::graspCallback, this);
  user_sync_server_ = nh_.advertiseService("user_sync", &DemoInterface::userSyncCallback, this);

  equilibrium_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose", 10);

  move_to_contact_server_ = new actionlib::SimpleActionServer<panda_pbd::MoveToContactAction>(
          nh_, "move_to_contact_server",boost::bind(&DemoInterface::moveToContactCallback, this, _1), false);
  move_to_contact_server_->start();

  // Clients
  cartesian_impedance_dynamic_reconfigure_client_ = nh_.
      serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");

  cartesian_impedance_direction_dynamic_reconfigure_client_ = nh_.
          serviceClient<dynamic_reconfigure::Reconfigure>
                  ("/dynamic_reconfigure_cartesian_impedance_direction_param_node/set_parameters");

  forcetorque_collision_client_ = nh_.
                                  serviceClient<franka_control::SetForceTorqueCollisionBehavior>
                                          ("/franka_control/set_force_torque_collision_behavior");
  controller_manager_switch_ = nh_.
          serviceClient<controller_manager_msgs::SwitchController>
                  ("/controller_manager/switch_controller");

  gripper_grasp_client_ = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", true);

  gripper_move_client_ = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("/franka_gripper/move", true);

  ROS_INFO("Waiting for the controller manager");
  controller_manager_switch_.waitForExistence();

  ROS_INFO("Starting the Impedance Controller...");
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.start_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_controller.request.strictness = 2;

  // TODO: proper handling here please
  bool loaded_controller = false;
  while (!loaded_controller){
    controller_manager_switch_.call(switch_controller);
    loaded_controller = switch_controller.response.ok;
    if (loaded_controller){
      ROS_INFO("Impedance Controller started");
    } else {
      ROS_ERROR("Impedance Controller not started. Trying again in 2 seconds...");
      ros::Duration(2).sleep();
    }
  }

  move_group_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  auto links = move_group_->getLinkNames();

  ROS_INFO("WHAT IS AVAILABLE!?");
  for(auto it = links.begin(); it != links.end(); it++)    {
    ROS_INFO("%s", it->c_str());
  }

}

geometry_msgs::PoseStamped DemoInterface::getEEPose()
{
  std::string tf_err_msg;
  std::string refFrame = BASE_FRAME;
  std::string childFrame = EE_FRAME;
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

geometry_msgs::PoseStamped DemoInterface::getHandPose()
{
  std::string tf_err_msg;
  std::string refFrame = BASE_FRAME;
  std::string childFrame = HAND_FRAME;
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

bool DemoInterface::adjustDirectionControllerParameters(geometry_msgs::Vector3 direction,
                                                        double speed = 0.0,
                                                        double transl_stiff = 1000.0,
                                                        double rotat_stiff = 200.0,
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

  ROS_INFO("Changing %s parameters...", DIRECTION_CONTROLLER.c_str());
  return cartesian_impedance_direction_dynamic_reconfigure_client_.call(direction_srv);
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
    case 4: {
      ROS_WARN("This is a moveit test and should be removed");
      ROS_WARN("Did it manage? %s", testPlanning() ? "yes" : "no");
      result = true;
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

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
    panda_pbd::EnableTeaching::Response &res)
{
  res.success = adjustImpedanceControllerStiffness(req,res);
  return res.success;
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

  adjustImpedanceControllerStiffness(1000.0, 200.0, 10.0);
  ROS_WARN("Setting the robot to be stiff (to be pushable)");

  while (!threshold_exceeded)
  {
    /* The external wrench is in the EE frame
     * Positive value when the force is applied AGAINST the axis
     */
    last_external_wrench_ptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
            "/franka_state_controller/F_ext");

    if (last_external_wrench_ptr != nullptr)
    {
      last_wrench_ = *last_external_wrench_ptr;
      if (std::abs(last_wrench_.wrench.force.x) > req.force_threshold.x ||
          std::abs(last_wrench_.wrench.force.y) > req.force_threshold.y ||
          std::abs(last_wrench_.wrench.force.z) > req.force_threshold.z)
      {
        ROS_INFO("Robot unlocked!");
        ROS_INFO("[%f %f %f] sensed",
                last_wrench_.wrench.force.x,
                last_wrench_.wrench.force.y,
                last_wrench_.wrench.force.z);
        threshold_exceeded = true;
      } else {
        ROS_INFO_THROTTLE(10, "Not enough force ([%f %f %f] sensed)",
                last_wrench_.wrench.force.x,
                last_wrench_.wrench.force.y,
                last_wrench_.wrench.force.z);
      }
    }
  }
  res.success = true;
  // setting back to default
  ROS_DEBUG("Setting stiffness back to standard values");
  adjustImpedanceControllerStiffness(200.0, 10.0, 1.0);
  return res.success;
}

void DemoInterface::moveToContactCallback(const panda_pbd::MoveToContactGoalConstPtr &goal){
  // TODO: do proper error handling
  ROS_INFO("Received MoveToContact request");

  ROS_INFO("Trying to switch to the direction controller...");
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.stop_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_controller.request.start_controllers.push_back(DIRECTION_CONTROLLER);
  switch_controller.request.strictness = 2;

  controller_manager_switch_.call(switch_controller);
  ROS_INFO("and... %s", switch_controller.response.ok ? "SUCCESS" : "FAILURE");
  if (!switch_controller.response.ok)
    return;

  ROS_WARN("Setting the robot to be stiff (to be pushable)");
  adjustDirectionControllerParameters(goal.get()->direction, goal.get()->speed);

  ROS_INFO("Direction Controller until contact...");

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
        ROS_INFO("Robot touched something!");
        ROS_INFO("[%f %f %f] sensed",
                 last_wrench_.wrench.force.x,
                 last_wrench_.wrench.force.y,
                 last_wrench_.wrench.force.z);
        in_contact = true;
      } else {
        ROS_INFO_THROTTLE(10, "Not enough force ([%f %f %f] sensed)",
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

  ROS_INFO("Trying to switch back to the impedance controller...");
  controller_manager_msgs::SwitchController switch_back_controller;
  switch_back_controller.request.start_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_back_controller.request.stop_controllers.push_back(DIRECTION_CONTROLLER);
  switch_back_controller.request.strictness = 2;

  adjustImpedanceControllerStiffness();
  controller_manager_switch_.call(switch_back_controller);
  ROS_INFO("and... %s", switch_back_controller.response.ok ? "SUCCESS" : "FAILURE");
  if (!switch_back_controller.response.ok)
    return;
}

bool DemoInterface::testPlanning() {
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  ROS_INFO("Trying to switch to the joint controller...");
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.stop_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_controller.request.start_controllers.push_back(JOINT_CONTROLLER);
  switch_controller.request.strictness = 2;

  controller_manager_switch_.call(switch_controller);
  ROS_INFO("and... %s", switch_controller.response.ok ? "SUCCESS" : "FAILURE");
  if (!switch_controller.response.ok)
    return false;


  geometry_msgs::PoseStamped current_pose = getHandPose();
  ROS_WARN("[%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  geometry_msgs::Pose target_pose;

  target_pose.orientation = current_pose.pose.orientation;
  target_pose.position = current_pose.pose.position;
  target_pose.position.z += 0.04; // move 4 cm up
  target_pose.position.y += 0.04; // move 4 cm up

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group_->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_->getEndEffectorLink().c_str());

  ROS_WARN("[%f, %f, %f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);

  move_group_->setPoseTarget(target_pose);

  ROS_INFO("Trying to plan");
  bool result = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Planning? %s", result ? "SUCCESS" : "FAILED");

  move_group_->move();


  ROS_INFO("Trying to switch to the impedance controller...");
  controller_manager_msgs::SwitchController switch_back_controller;
  switch_back_controller.request.stop_controllers.push_back(JOINT_CONTROLLER);
  switch_back_controller.request.start_controllers.push_back(IMPEDANCE_CONTROLLER);
  switch_back_controller.request.strictness = 2;

  controller_manager_switch_.call(switch_back_controller);
  ROS_INFO("and... %s", switch_back_controller.response.ok ? "SUCCESS" : "FAILURE");
  if (!switch_back_controller.response.ok)
    return false;
}
