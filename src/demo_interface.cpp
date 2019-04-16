#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface(): nh_("~")
{
    kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
}

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
                                                panda_pbd::EnableTeaching::Response &res)
{
    if (req.teaching)
    {
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

        double_param.name = "translational_stiffness";
        double_param.value = 0.0;

        conf.doubles.push_back(double_param);
        srv_req.config = conf;

        ros::service::call("/dynamic_reconfigure_compliance_param_node/set_parameters", srv_req, srv_resp);
    } else {
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::DoubleParameter double_param;
        dynamic_reconfigure::Config conf;

        double_param.name = "translational_stiffness";
        double_param.value = 200.0;

        conf.doubles.push_back(double_param);
        srv_req.config = conf;

        ros::service::call("/dynamic_reconfigure_compliance_param_node/set_parameters", srv_req, srv_resp);
    }
}
