#!/usr/bin/env python

import rospy
from panda_pbd.pbd_interface import PandaPBDInterface

if __name__ == "__main__":
    rospy.init_node('panda_pbd_interface_node', anonymous=True)
    ppi = PandaPBDInterface()
    rospy.spin()
