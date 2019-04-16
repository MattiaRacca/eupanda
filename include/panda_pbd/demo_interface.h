#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"

// Custom services
#include "panda_pbd/EnableTeaching.h"

class DemoInterface
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer kinesthetic_server_;
    ros::ServiceClient cartesian_impedance_dynamic_reconfigure_client_;
    ros::Publisher equilibrium_pose_publisher_;
    tf::TransformListener pose_listener_;

    bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
    geometry_msgs::PoseStamped getEEPose();
public:
    DemoInterface();
};

#endif // DEMO_INTERFACE_H