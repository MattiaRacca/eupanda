#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface(): nh_("~")
{
    kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
    joint_stiffness_client_ = nh_.serviceClient<franka_control::SetJointImpedance>("set_joint_impedance");
}

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
                                                panda_pbd::EnableTeaching::Response &res)
{
    if (req.teaching)
    {
        boost::array<double, 7> stiffness;
        stiffness.fill(0);
        franka_control::SetJointImpedance srv;
        srv.request.joint_stiffness = stiffness;
        if (joint_stiffness_client_.call(srv))
        {
            ROS_INFO("%d", srv.response.success);
            res.response = 1;
        } else {
            ROS_ERROR("Failed to call SetJointImpedance service");
            ROS_ERROR("%s", srv.response.error.c_str());
            res.response = -1;
        }
    }
}
