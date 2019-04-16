#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

// Custom services
#include "panda_pbd/EnableTeaching.h"

class DemoInterface
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer kinesthetic_server_;
    ros::ServiceClient cartesian_impedance_dynamic_reconfigure_client_;
    bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
public:
    DemoInterface();
};

#endif // DEMO_INTERFACE_H