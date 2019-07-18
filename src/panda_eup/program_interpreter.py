#!/usr/bin/env python

import rospy
import actionlib
import panda_primitive as pp

from panda_pbd.msg import UserSyncAction, MoveToContactAction, MoveToEEAction
from panda_pbd.srv import MoveFingers, ApplyForceFingers

from panda_pbd.msg import MoveToEEGoal
from panda_pbd.srv import MoveFingersRequest, ApplyForceFingersRequest
from panda_pbd.srv import MoveFingersResponse, ApplyForceFingersResponse
from std_msgs.msg import Int32

class PandaProgramInterpreter(object):
    def __init__(self, robotless_debug=False):
        self.loaded_program = None
        self.next_primitive_index = -1  # next primitive to be executed!
        self.last_panda_status = None

        self.robotless_debug = robotless_debug
        self.fake_wait = 3

        self.revert_default_position_speed = .04  # m/s
        self.revert_default_rotation_speed = 1.0  # rad/s

        # TO THE PRIMITIVE_INTERFACE, IF ANY
        if not self.robotless_debug:
            self.panda_state_client = rospy.Subscriber("/primitive_interface_node/interface_state", Int32,
                                                       self.panda_state_callback)

            self.move_to_ee_client = actionlib.SimpleActionClient( '/primitive_interface_node/move_to_ee_server',
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
        else:
            self.last_panda_status = pp.PandaRobotStatus.READY

        # PRIMITIVE CALLBACKS (FORWARD AND REVERSE)
        self.callback_switcher = {
            pp.UserSync: self.execute_user_sync,
            pp.MoveToContact: self.execute_move_to_contact,
            pp.MoveToEE: self.execute_move_to_ee,
            pp.MoveFingers: self.execute_move_fingers,
            pp.ApplyForceFingers: self.execute_apply_force_fingers
        }

        self.revert_callback_switcher = {
            pp.UserSync: self.revert_user_sync,
            pp.MoveToContact: self.revert_move_to_contact,
            pp.MoveToEE: self.revert_move_to_ee,
            pp.MoveFingers: self.revert_move_fingers,
            pp.ApplyForceFingers: self.revert_apply_force_fingers
        }

    def panda_state_callback(self, msg):
        new_panda_status = pp.PandaRobotStatus(msg.data)
        if self.last_panda_status != new_panda_status:
            self.last_panda_status = new_panda_status

    def __str__(self):
        if self.loaded_program is None:
            return 'No program loaded'
        full_description = self.loaded_program.__str__()
        try:
            full_description += 'Currently ready to execute primitive {}: '.format(self.next_primitive_index) + \
                                self.loaded_program.get_nth_primitive(self.next_primitive_index).__str__()
        except pp.PandaProgramException:
            full_description += 'Interpreter instruction pointer {} out of program range!'. \
                format(self.next_primitive_index)

        return full_description

    def load_program(self, program):
        self.loaded_program = program
        self.next_primitive_index = -1
        for primitive in program.primitives:
            primitive.status = pp.PandaPrimitiveStatus.NEUTRAL

    def go_to_starting_state(self, progress_callback=None):
        if self.loaded_program is None:
            return False

        try:
            arm_state, gripper_state = self.loaded_program.get_nth_primitive_preconditions(0)
        except pp.PandaProgramException:
            rospy.logerr('Cannot find starting conditions: did you save the starting state to begin with?')
            return False

        if not self.robotless_debug:
            # create new Goal for the primitive
            goal = MoveToEEGoal()
            goal.pose = arm_state
            goal.position_speed = self.revert_default_position_speed
            goal.rotation_speed = self.revert_default_rotation_speed

            self.move_to_ee_client.send_goal(goal)

            if progress_callback is not None:
                progress_callback.emit(self.next_primitive_index)

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
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True
            success_arm = True

        if success_arm and response.success:
            self.next_primitive_index = 0
            for primitive in self.loaded_program.primitives:
                primitive.status = pp.PandaPrimitiveStatus.NEUTRAL

        return success_arm and response.success

    def execute_one_step(self, progress_callback=None):
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

        primitive_to_execute.status = pp.PandaPrimitiveStatus.EXECUTING
        if progress_callback is not None:
            progress_callback.emit(self.next_primitive_index)
        result = callback(primitive_to_execute)

        if result:
            rospy.loginfo('Executed primitive ' + primitive_to_execute.__str__())
            primitive_to_execute.status = pp.PandaPrimitiveStatus.EXECUTED
            self.next_primitive_index += 1
        else:
            rospy.logerr('Error while executing ' + primitive_to_execute.__str__())
            primitive_to_execute.status = pp.PandaPrimitiveStatus.ERROR
            return False

        return True

    def revert_one_step(self, progress_callback=None):
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
            primitive_to_revert.status = pp.PandaPrimitiveStatus.ERROR
            return False

        callback = self.revert_callback_switcher.get(primitive_to_revert.__class__, None)

        if callback is None:
            rospy.logwarn("I don't know how to revert this primitive; did you define the revert callback?")
            return False

        primitive_to_revert.status = pp.PandaPrimitiveStatus.REVERTING
        if progress_callback is not None:
            progress_callback.emit(self.next_primitive_index)
        result = callback(self.next_primitive_index - 1)

        if result:
            rospy.loginfo('Reverted primitive ' + primitive_to_revert.__str__())
            primitive_to_revert.status = pp.PandaPrimitiveStatus.NEUTRAL
            self.next_primitive_index -= 1
        else:
            rospy.logerr('Error while reverting ' + primitive_to_revert.__str__())
            primitive_to_revert.status = pp.PandaPrimitiveStatus.ERROR
            return False

        return True

    def execute_rest_of_program(self, one_shot_execution=False, progress_callback=None):
        partial_success = True
        if one_shot_execution:
            # TODO: this is just for the execute now (like relax_fingers and execute_primitive_now).
            #   Find a better way to do this
            to_be_restored_index = self.next_primitive_index
            self.next_primitive_index = 0
        primitive_counter = self.next_primitive_index

        while partial_success:
            partial_success = self.execute_one_step(progress_callback)
            if partial_success:
                primitive_counter += 1

        result = False
        if primitive_counter == self.loaded_program.get_program_length():
            rospy.loginfo('Executed rest of the program!')
            result = True
        else:
            rospy.logerr('Something went south. Program now at step {}'.format(self.next_primitive_index))
            result = False
        if one_shot_execution:
            self.next_primitive_index = to_be_restored_index

        return result

    def revert_to_beginning_of_program(self, progress_callback=None):
        partial_success = True
        primitive_counter = self.next_primitive_index

        while partial_success:
            partial_success = self.revert_one_step(progress_callback)
            if partial_success:
                primitive_counter -= 1

        if primitive_counter == 0:
            # TODO: maybe this should be handled by revert_one_step?
            self.go_to_starting_state()
            rospy.loginfo('Reverted to beginning of the program!')
            return True
        else:
            rospy.logerr('Something went south while reverting. Program now at step {}'.format(
                self.next_primitive_index))
            return False

    # PRIMITIVE CALLBACKS
    def execute_user_sync(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a user sync')
        if not self.robotless_debug:
            self.user_sync_client.send_goal(primitive_to_execute.parameter_container)
            success = self.user_sync_client.wait_for_result()
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            success = True

        rospy.loginfo('Success? ' + str(success))
        return success

    def execute_move_to_contact(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a move to contact')

        if not self.robotless_debug:
            self.move_to_contact_client.send_goal(primitive_to_execute.parameter_container)
            success = self.move_to_contact_client.wait_for_result(rospy.Duration(60))  # TODO: questionable choice?

            state = self.move_to_contact_client.get_state()

            if state == actionlib.GoalStatus.ABORTED or not success:
                success = False
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            success = True

        rospy.loginfo('Success? ' + str(success))
        return success

    def execute_move_to_ee(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a move to EE')

        if not self.robotless_debug:
            self.move_to_ee_client.send_goal(primitive_to_execute.parameter_container)
            success = self.move_to_ee_client.wait_for_result(rospy.Duration(60))

            state = self.move_to_contact_client.get_state()

            if state == actionlib.GoalStatus.ABORTED or not success:
                success = False
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            success = True

        rospy.loginfo('Success? ' + str(success))
        return success

    def execute_move_fingers(self, primitive_to_execute):
        rospy.loginfo('Trying to execute a move fingers')
        if not self.robotless_debug:
            response = self.move_fingers_client.call(primitive_to_execute.parameter_container)
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    def execute_apply_force_fingers(self, primitive_to_execute):
        rospy.loginfo('Trying to execute an apply force with fingers')
        if not self.robotless_debug:
            response = self.apply_force_fingers_client.call(primitive_to_execute.parameter_container)
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = ApplyForceFingersResponse()
            response.success = True
        rospy.loginfo('Success? :' + str(response.success))
        return response.success

    # REVERT PRIMITIVE CALLBACK
    def revert_user_sync(self, primitive_index):
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

        if not self.robotless_debug:
            self.move_to_ee_client.send_goal(goal)
            success = self.move_to_ee_client.wait_for_result()
            state = self.move_to_contact_client.get_state()

            if state == actionlib.GoalStatus.ABORTED or not success:
                success = False
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            success = True

        rospy.loginfo('Success? ' + str(success))
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

        if not self.robotless_debug:
            self.move_to_ee_client.send_goal(goal)
            success = self.move_to_ee_client.wait_for_result()
            state = self.move_to_contact_client.get_state()

            if state == actionlib.GoalStatus.ABORTED or not success:
                success = False
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            success = True

        rospy.loginfo('Success? ' + str(success))
        return success

    def revert_move_fingers(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.loginfo('Trying to revert a move fingers to {}, {}'.format(gripper_state.width, gripper_state.force))

        if not self.robotless_debug:
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
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True

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

        if not self.robotless_debug:
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
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True

        rospy.loginfo('Success? :' + str(response.success))
        return response.success
