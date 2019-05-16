#!/usr/bin/env python

import rospy
import actionlib
import panda_primitive as pp

from panda_pbd.msg import UserSyncAction, MoveToContactAction, MoveToEEAction
from panda_pbd.srv import OpenGripper, CloseGripper

from panda_pbd.msg import MoveToEEGoal
from panda_pbd.srv import OpenGripperRequest


class PandaProgramInterpreter(object):
    def __init__(self):
        self.loaded_program = None
        self.next_primitive_index = 0  # next primitive to be executed!

        self.revert_default_position_speed = .04  # m/s
        self.revert_default_rotation_speed = 1.0  # rad/s

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

        # PRIMITIVE CALLBACKS (FORWARD AND REVERSE)
        self.callback_switcher = {
            pp.OpenGripper: self.execute_open_gripper,
            pp.CloseGripper: self.execute_close_gripper,
            pp.UserSync: self.execute_user_sync,
            pp.MoveToContact: self.execute_move_to_contact,
            pp.MoveToEE: self.execute_move_to_ee
        }

        self.revert_callback_switcher = {
            pp.OpenGripper: self.revert_open_gripper,
            pp.CloseGripper: self.revert_close_gripper,
            pp.UserSync: self.revert_user_sync,
            pp.MoveToContact: self.revert_move_to_contact,
            pp.MoveToEE: self.revert_move_to_ee
        }

    def __str__(self):
        if self.loaded_program is None:
            return 'No program loaded'
        full_description = self.loaded_program.__str__()
        full_description += 'Currently ready to execute primitive {}:'.format(self.next_primitive_index) + \
                            self.loaded_program.get_nth_primitive(self.next_primitive_index).__str__()

        return full_description

    def load_program(self, program):
        self.loaded_program = program
        self.next_primitive_index = 0

    def go_to_starting_state(self):
        if self.loaded_program is None:
            return False

    def execute_one_step(self):
        if self.loaded_program is None:
            rospy.logwarn('no program loaded')
            return False

        primitive_to_execute = self.loaded_program.get_nth_primitive(self.next_primitive_index)

        if primitive_to_execute is None:
            rospy.logwarn('Nothing left to execute OR Empty program OR wrong indexing')
            return False

        callback = self.callback_switcher.get(primitive_to_execute.__class__, None)

        if callback is None:
            rospy.logwarn("I don't know how to execute this primitive; did you define the callback?")
            return False

        result = callback(primitive_to_execute)

        if result:
            rospy.loginfo('Executed primitive ' + primitive_to_execute.__str__())
            self.next_primitive_index += 1
        else:
            rospy.logerr('Error while executing ' + primitive_to_execute.__str__())
            return False

        return True

    def revert_one_step(self):
        if self.loaded_program is None:
            rospy.logwarn('no program loaded')
            return False

        primitive_to_revert = self.loaded_program.get_nth_primitive(self.next_primitive_index - 1)

        if primitive_to_revert is None:
            rospy.logwarn('Nothing left to revert OR Empty program OR wrong indexing')
            return False

        callback = self.callback_switcher.get(primitive_to_revert.__class__, None)

        if callback is None:
            rospy.logwarn("I don't know how to revert this primitive; did you define the revert callback?")
            return False

        result = callback(primitive_to_revert)

        if result:
            rospy.loginfo('Reverted primitive ' + primitive_to_revert.__str__())
            self.next_primitive_index -= 1
        else:
            rospy.logerr('Error while reverting ' + primitive_to_revert.__str__())
            return False

        return True

    def execute_rest_of_program(self):
        partial_success = True
        primitive_counter = self.next_primitive_index

        while partial_success:
            partial_success = self.execute_one_step()
            if partial_success:
                primitive_counter += 1

        if primitive_counter == self.loaded_program.get_program_length() - 1:
            rospy.loginfo('Executed rest of the program!')
            return True
        else:
            rospy.logerr('Something went south. Program now at step {}'.format(self.next_primitive_index))
            return False

    def revert_to_beginning_of_program(self):
        partial_success = True
        primitive_counter = self.next_primitive_index

        while partial_success:
            partial_success = self.revert_one_step()
            if partial_success:
                primitive_counter -= 1

        if primitive_counter == 0:
            rospy.loginfo('Reverted to beginning of the program!')
            # TODO: check here value of next_primitive_index
            rospy.logwarn('Should the next_primitive_index be 0? now it is {}'.format(self.next_primitive_index))
            return True
        else:
            rospy.logerr('Something went south while reverting. Program now at step {}'.format(self.next_primitive_index))
            return False

    ### PRIMITIVE CALLBACKS
    def execute_open_gripper(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a open gripper')
        response = self.open_gripper_client.call(primitive_to_execute.parameter_container)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def execute_close_gripper(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a close gripper')
        response = self.close_gripper_client.call(primitive_to_execute.parameter_container)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def execute_user_sync(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a user sync')
        self.user_sync_client.send_goal(primitive_to_execute.parameter_container)
        success = self.user_sync_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success

    def execute_move_to_contact(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a move to contact')
        self.move_to_contact_client.send_goal(primitive_to_execute.parameter_container)
        success = self.move_to_contact_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success

    def execute_move_to_ee(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a move to EE')
        self.move_to_ee_client.send_goal(primitive_to_execute.parameter_container)
        success = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success

    ## REVERT PRIMITIVE CALLBACK
    def revert_open_gripper(self, primitive_to_revert):
        rospy.loginfo('Trying to revert a open gripper')
        pose, width = primitive_to_revert.get_nth_primitive_preconditions()

        request = OpenGripperRequest()
        request.width = width

        # TODO: this is probably not enough to revert!
        response = self.open_gripper_client.call(request)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def revert_close_gripper(self, primitive_to_revert):
        rospy.loginfo('Trying to revert a close gripper')
        pose, width = primitive_to_revert.get_nth_primitive_preconditions()

        request = OpenGripperRequest()
        request.width = width

        response = self.open_gripper_client.call(request)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def revert_user_sync(self, primitive_to_revert):
        rospy.loginfo('Trying to revert a user sync')
        # User Sync does not require robot motions to be reset
        success = True
        rospy.loginfo('Success? :' + str(success))
        return success

    def revert_move_to_contact(self, primitive_to_revert):
        rospy.loginfo('Trying to revert a move to contact')
        pose, width = primitive_to_revert.get_nth_primitive_preconditions()

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        self.move_to_ee_client.send_goal(goal)
        success = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success

    def revert_move_to_ee(self, primitive_to_revert):
        rospy.loginfo('Trying to revert a move to EE')
        pose, width = primitive_to_revert.get_nth_primitive_preconditions()

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        self.move_to_ee_client.send_goal(goal)
        success = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success
