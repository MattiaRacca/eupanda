#!/usr/bin/env python

import rospy
import actionlib
import panda_primitive as pp
from panda_pbd.msg import UserSyncAction, MoveToContactAction, MoveToEEAction
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import OpenGripper, CloseGripper
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest


class PandaProgramInterpreter(object):
    def __init__(self):
        self.current_program = None
        self.current_step = 0

        # TO THE PRIMITIVE_INTERFACE
        self.move_to_ee_client = actionlib.SimpleActionClient('/primitive_interface_node/move_to_ee_server',
                                                              MoveToEEAction)
        self.move_to_contact_client = actionlib.SimpleActionClient('primitive_interface_node/move_to_contact_server',
                                                                   MoveToContactAction)
        self.user_sync_client = actionlib.SimpleActionClient('/primitive_interface_node/user_sync_server',
                                                             UserSyncAction)

        self.move_to_ee_client.wait_for_server()
        self.move_to_contact_client.wait_for_server()
        self.user_sync_client.wait_for_server()

        self.open_gripper_client = rospy.ServiceProxy('/primitive_interface_node/open_gripper', OpenGripper)
        self.close_gripper_client = rospy.ServiceProxy('/primitive_interface_node/close_gripper', CloseGripper)

        self.open_gripper_client.wait_for_service()
        self.close_gripper_client.wait_for_service()
