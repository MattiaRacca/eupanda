#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

#include <ros/ros.h>
#include <franka_control/SetJointImpedance.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "panda_pbd/EnableTeaching.h"

class DemoInterface
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer kinesthetic_server_;
    bool kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req, panda_pbd::EnableTeaching::Response &res);
public:
    DemoInterface();
};

#endif // DEMO_INTERFACE_H