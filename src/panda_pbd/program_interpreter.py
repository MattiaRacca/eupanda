#!/usr/bin/env python

import rospy
import actionlib
import panda_primitive as pp

from panda_pbd.msg import UserSyncAction, MoveToContactAction, MoveToEEAction
from panda_pbd.srv import MoveFingers, ApplyForceFingers, OpenGripper, CloseGripper

from panda_pbd.msg import MoveToEEGoal
from panda_pbd.srv import MoveFingersRequest, ApplyForceFingersRequest, OpenGripperRequest


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

        self.move_fingers_client = rospy.ServiceProxy('/primitive_interface_node/move_fingers', MoveFingers)
        self.apply_force_fingers_client = rospy.ServiceProxy('/primitive_interface_node/apply_force_fingers',
                                                             ApplyForceFingers)

        self.open_gripper_client = rospy.ServiceProxy('/primitive_interface_node/open_gripper', OpenGripper)
        self.close_gripper_client = rospy.ServiceProxy('/primitive_interface_node/close_gripper', CloseGripper)

        self.open_gripper_client.wait_for_service()
        self.close_gripper_client.wait_for_service()

        # PRIMITIVE CALLBACKS (FORWARD AND REVERSE)
        self.callback_switcher = {
            pp.UserSync: self.execute_user_sync,
            pp.MoveToContact: self.execute_move_to_contact,
            pp.MoveToEE: self.execute_move_to_ee,
            pp.MoveFingers: self.execute_move_fingers,
            pp.ApplyForceFingers: self.execute_apply_force_fingers,
            pp.OpenGripper: self.execute_open_gripper,
            pp.CloseGripper: self.execute_close_gripper
        }

        self.revert_callback_switcher = {
            pp.UserSync: self.revert_user_sync,
            pp.MoveToContact: self.revert_move_to_contact,
            pp.MoveToEE: self.revert_move_to_ee,
            pp.MoveFingers: self.revert_move_fingers,
            pp.ApplyForceFingers: self.revert_apply_force_fingers,
            pp.OpenGripper: self.revert_open_gripper,
            pp.CloseGripper: self.revert_close_gripper
        }

    def __str__(self):
        if self.loaded_program is None:
            return 'No program loaded'
        full_description = self.loaded_program.__str__()
        try:
            full_description += 'Currently ready to execute primitive {}:'.format(self.next_primitive_index) + \
                            self.loaded_program.get_nth_primitive(self.next_primitive_index).__str__()
        except pp.PandaProgramException:
            full_description += 'Interpreter instruction pointer {} out of program range!'.\
                format(self.next_primitive_index)

        return full_description

    def load_program(self, program):
        self.loaded_program = program
        self.next_primitive_index = 0

    def go_to_starting_state(self):
        if self.loaded_program is None:
            return False

        try:
            arm_state, gripper_state = self.loaded_program.get_nth_primitive_preconditions(0)
        except pp.PandaProgramException:
            rospy.logerr('Cannot find starting conditions: did you save the starting state to begin with?')
            return False

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = arm_state
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        self.move_to_ee_client.send_goal(goal)
        success_arm = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success_arm))

        if gripper_state.force > 0.0:
            # need to revert to a apply_force_fingers
            request = ApplyForceFingersRequest()
            request.force = gripper_state.force
            response = self.apply_force_fingers_client.call(request)
        else:
            # need to revert to a move_fingers
            request = MoveFingersRequest()
            request.width = gripper_state.width
            response = self.move_fingers_client.call(request)

        rospy.loginfo('Success? :' + str(response.success))

        return success_arm and response.success

    def execute_one_step(self):
        if self.loaded_program is None:
            rospy.logwarn('no program loaded')
            return False

        try:
            primitive_to_execute = self.loaded_program.get_nth_primitive(self.next_primitive_index)
        except pp.PandaProgramException:
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

        try:
            primitive_to_revert = self.loaded_program.get_nth_primitive(self.next_primitive_index - 1)
        except pp.PandaProgramException:
            rospy.logwarn('Nothing left to revert OR Empty program OR wrong indexing')
            return False

        if not primitive_to_revert.revertible:
            rospy.logwarn('Cannot revert this primitive! the previous primitives was not completely updated...')
            return False

        callback = self.revert_callback_switcher.get(primitive_to_revert.__class__, None)

        if callback is None:
            rospy.logwarn("I don't know how to revert this primitive; did you define the revert callback?")
            return False

        result = callback(self.next_primitive_index - 1)

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

        if primitive_counter == self.loaded_program.get_program_length():
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
            return True
        else:
            rospy.logerr('Something went south while reverting. Program now at step {}'.format(
                self.next_primitive_index))
            return False

    # PRIMITIVE CALLBACKS
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

    def execute_move_fingers(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a move fingers')
        response = self.move_fingers_client.call(primitive_to_execute.parameter_container)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def execute_apply_force_fingers(self, primitive_to_execute):
        rospy.loginfo('Trying to execute an apply force with fingers')
        response = self.apply_force_fingers_client.call(primitive_to_execute.parameter_container)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def execute_open_gripper(self, primitive_to_execute):
        # LEGACY
        rospy.loginfo('Trying to execute a open gripper')
        response = self.open_gripper_client.call(primitive_to_execute.parameter_container)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def execute_close_gripper(self, primitive_to_execute):
        # LEGACY
        rospy.loginfo('Trying to execute a close gripper')
        response = self.close_gripper_client.call(primitive_to_execute.parameter_container)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    # REVERT PRIMITIVE CALLBACK
    def revert_user_sync(self, primitive_to_revert):
        rospy.loginfo('Trying to revert a user sync')
        # User Sync does not require robot motions to be reset
        success = True
        rospy.loginfo('Success? :' + str(success))
        return success

    def revert_move_to_contact(self, primitive_index):
        # TODO: reverting something AFTER a move_to_contact might not be that easy...
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert a move to contact')

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        self.move_to_ee_client.send_goal(goal)
        success = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success

    def revert_move_to_ee(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert a move to EE')

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        self.move_to_ee_client.send_goal(goal)
        success = self.move_to_ee_client.wait_for_result()
        rospy.loginfo('Success? :' + str(success))
        return success

    def revert_move_fingers(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert a move fingers to {}, {}'.format(gripper_state.width, gripper_state.force))

        if gripper_state.force > 0.0:
            # need to revert to a apply_force_fingers
            request = ApplyForceFingersRequest()
            request.force = gripper_state.force
            response = self.apply_force_fingers_client.call(request)
        else:
            # need to revert to a move_fingers
            request = MoveFingersRequest()
            request.width = gripper_state.width
            response = self.move_fingers_client.call(request)

        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def revert_apply_force_fingers(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert an apply force fingers to {}, {}'.format(gripper_state.width,
                                                                                 gripper_state.force))

        if gripper_state.force > 0.0:
            # need to revert to a apply_force_fingers
            request = ApplyForceFingersRequest()
            request.force = gripper_state.force
            response = self.apply_force_fingers_client.call(request)
        else:
            # need to revert to a move_fingers
            request = MoveFingersRequest()
            request.width = gripper_state.width
            response = self.move_fingers_client.call(request)

        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def revert_open_gripper(self, primitive_index):
        # LEGACY
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert a open gripper to {}'.format(gripper_state.width))

        request = OpenGripperRequest()
        request.width = gripper_state.width

        # TODO: this is probably not enough to revert!
        response = self.open_gripper_client.call(request)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def revert_close_gripper(self, primitive_index):
        # LEGACY
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert a close gripper to {}'.format(gripper_state.width))

        request = OpenGripperRequest()
        request.width = gripper_state.width

        response = self.open_gripper_client.call(request)
        rospy.loginfo('Success? :' + str(response.success))
        return response.success
