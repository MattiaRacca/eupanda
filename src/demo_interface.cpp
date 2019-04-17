#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface(): nh_("~")
{
  kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
  cartesian_impedance_dynamic_reconfigure_client_ = nh_.
      serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");
  forcetorque_collision_client_ = nh_.
                                  serviceClient<franka_control::SetForceTorqueCollisionBehavior>("/franka_control/set_force_torque_collision_behavior");
  equilibrium_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose", 10);
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
bool DemoInterface::AdjustFTThreshold(double ft_multiplier)
{
  franka_control::SetForceTorqueCollisionBehavior collision_srv;

  boost::array<double, 6> force_threshold{ {20.0, 20.0, 20.0, 25.0, 25.0, 25.0} };
  boost::array<double, 7> torque_threshold{ {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0} };

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

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
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
      AdjustFTThreshold(1);
      translational_stiff.value = default_translation_stiffness;
      rotational_stiff.value = default_rotational_stiffness;
      break;
      }
    case 1: {
      ROS_INFO("Full Teaching mode activated...");
      AdjustFTThreshold(req.ft_threshold_multiplier);
      translational_stiff.value = 0.0;
      rotational_stiff.value = 0.0;
      break;
    }
    case 2: {
      ROS_INFO("Position Teaching mode activated...");
      AdjustFTThreshold(req.ft_threshold_multiplier);
      translational_stiff.value = 0.0;
      rotational_stiff.value = default_rotational_stiffness;
      break;
    }
    case 3: {
      ROS_INFO("Orientation Teaching mode activated...");
      AdjustFTThreshold(req.ft_threshold_multiplier);
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

  res.ee_pose = getEEPose();
  equilibrium_pose_publisher_.publish(res.ee_pose);
  ros::Duration(0.5).sleep(); // to allow the controller to receive the now equilibrium pose
  res.success = cartesian_impedance_dynamic_reconfigure_client_.call(stiffness_srv);

  return true;
}
