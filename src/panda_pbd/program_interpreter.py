#!/usr/bin/env python

import rospy
import actionlib
import panda_primitive as pp
from panda_pbd.msg import UserSyncAction, MoveToContactAction, MoveToEEAction
from panda_pbd.srv import OpenGripper, CloseGripper


class PandaProgramInterpreter(object):
    def __init__(self):
        self.current_program = None
        self.current_primitive = None
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

        self.callback_switcher = {
            pp.OpenGripper : self.execute_open_gripper,
            pp.CloseGripper : self.execute_close_gripper,
            pp.UserSync : self.execute_user_sync,
            pp.MoveToContact : self.execute_move_to_contact,
            pp.MoveToEE : self.execute_move_to_ee
        }

    def __str__(self):
        if self.current_program is None:
            return 'No program loaded'
        full_description = self.current_program.__str__()
        full_description += 'Current at primitive {}:'.format(self.current_step) +\
                            self.current_program.values()[self.current_step].__str__()

        return full_description

    def execute_entire_program(self):
        partial_success = True
        primitive_counter = 0

        while partial_success:
            partial_success = self.execute_one_step()
            if partial_success:
                primitive_counter += 1

        if primitive_counter == self.current_program.get_program_length():
            return True
        else:
            return False

    def execute_one_step(self):
        if self.current_program is None:
            rospy.logwarn('no program loaded')
            return False

        primitive_to_execute = self.current_program.get_nth_primitive(self.current_step)

        if primitive_to_execute is None:
            rospy.logwarn('nothing left to execute')
            return False

        callback = self.callback_switcher.get(primitive_to_execute.__class__, lambda: None)

        if callback is None:
            rospy.logwarn("I don't know how to execute this primitive; did you define the callback?")
            return False

        result = callback()

        if result:
            rospy.loginfo('Executed primitive ' + primitive_to_execute.__str__())
            self.current_step += 1
        else:
            rospy.logerr('Error while executing ' + primitive_to_execute.__str__())
            return False

        return True


    def execute_open_gripper(self):
        rospy.loginfo('Trying to execute a open gripper')
        response = self.open_gripper_client.call(self.current_primitive.container)
        rospy.loginfo('Success? :' + response.success)
        return response.success

    def execute_close_gripper(self):
        rospy.loginfo('Trying to execute a close gripper')
        response = self.close_gripper_client.call(self.current_primitive.container)
        rospy.loginfo('Success? :' + response.success)
        return response.success

    def execute_user_sync(self):
        rospy.loginfo('Trying to execute a user sync')
        self.user_sync_client.send_goal(self.current_primitive.container)
        success = self.user_sync_client.wait_for_result()
        rospy.loginfo('Success? :' + success)
        return success

    def execute_move_to_contact(self):
        rospy.loginfo('Trying to execute a move to contact')
        self.move_to_contact_client.send_goal(self.current_primitive.container)
        success = self.move_to_contact_client.wait_for_result()
        rospy.loginfo('Success? :' + success)
        return success

    def execute_move_to_ee(self):
        rospy.loginfo('Trying to execute a move to EE')
        self.move_to_ee_client.send_goal(self.current_primitive.container)
        success = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + success)
        return success
