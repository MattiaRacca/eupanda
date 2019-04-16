#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface(): nh_("~")
{
    kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
    cartesian_impedance_dynamic_reconfigure_client_ = nh_.
            serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");
    equilibrium_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose",10);
}

geometry_msgs::PoseStamped DemoInterface::getEEPose()
{
    std::string tf_err_msg;
    std::string refFrame = "panda_link0";
    std::string childFrame = "panda_EE";
    tf::StampedTransform transform;

    if (!pose_listener_.waitForTransform(refFrame, childFrame, ros::Time(0),
                                         ros::Duration(0.5), ros::Duration(0.01),
                                         &tf_err_msg)) {
        ROS_ERROR_STREAM("Unable to get pose from TF: " << tf_err_msg);
    } else {
        try {
            pose_listener_.lookupTransform(refFrame, childFrame,
                                           ros::Time(0), // get latest available
                                           transform);
        } catch (const tf::TransformException &e) {
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

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
                                                panda_pbd::EnableTeaching::Response &res)
{
    dynamic_reconfigure::Reconfigure srv;

    dynamic_reconfigure::DoubleParameter translational_stiff;
    dynamic_reconfigure::DoubleParameter rotational_stiff;
    translational_stiff.name = "translational_stiffness";
    rotational_stiff.name = "rotational_stiffness";

    if (req.teaching)
    {
        translational_stiff.value = 0.0;
        rotational_stiff.value = 0.0;

        srv.request.config.doubles.push_back(translational_stiff);
        srv.request.config.doubles.push_back(rotational_stiff);

        res.success = cartesian_impedance_dynamic_reconfigure_client_.call(srv);
        res.current_mode = 1;
        res.ee_pose = getEEPose();
    } else {
        translational_stiff.value = 200.0;
        rotational_stiff.value = 10.0;

        srv.request.config.doubles.push_back(translational_stiff);
        srv.request.config.doubles.push_back(rotational_stiff);

        res.ee_pose = getEEPose();
        equilibrium_pose_publisher_.publish(res.ee_pose);
        res.success = cartesian_impedance_dynamic_reconfigure_client_.call(srv);
        res.current_mode = 0;
    }
    return true;
}
