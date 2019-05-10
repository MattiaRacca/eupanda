#include "panda_pbd/primitive_interface.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "primitive_interface_node");
  PrimitiveInterface node;

  // The MultiThreadedSpinner allows for multiple callbacks at the same time
  // TODO: proper thread-safe state machine...
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}