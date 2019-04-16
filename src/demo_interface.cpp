#include "panda_pbd/demo_interface.h"

DemoInterface::DemoInterface(): nh_("~")
{
    kinesthetic_server_ = nh_.advertiseService("kinesthetic_teaching", &DemoInterface::kinestheticTeachingCallback, this);
    cartesian_impedance_dynamic_reconfigure_client_ = nh_.
            serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_reconfigure_compliance_param_node/set_parameters");
}

bool DemoInterface::kinestheticTeachingCallback(panda_pbd::EnableTeaching::Request &req,
                                                panda_pbd::EnableTeaching::Response &res)
{
    dynamic_reconfigure::Reconfigure srv;

    dynamic_reconfigure::DoubleParameter translational_stiff;
    dynamic_reconfigure::DoubleParameter rotational_stiff;

    if (req.teaching)
    {
        translational_stiff.name = "translational_stiffness";
        translational_stiff.value = 0.0;
        rotational_stiff.name = "rotational_stiffness";
        rotational_stiff.value = 0.0;

        srv.request.config.doubles.push_back(translational_stiff);
        srv.request.config.doubles.push_back(rotational_stiff);

        res.success = cartesian_impedance_dynamic_reconfigure_client_.call(srv);
        res.current_mode = 1;
    } else {
        translational_stiff.name = "translational_stiffness";
        translational_stiff.value = 200.0;
        rotational_stiff.name = "rotational_stiffness";
        rotational_stiff.value = 10.0;

        srv.request.config.doubles.push_back(translational_stiff);
        srv.request.config.doubles.push_back(rotational_stiff);

        // TODO: update pose commanded to the cartesian_impedance_controller
        res.success = cartesian_impedance_dynamic_reconfigure_client_.call(srv);
        res.current_mode = 0;
    }

    return true;
}
