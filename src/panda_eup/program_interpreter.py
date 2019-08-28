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
from sensor_msgs.msg import JointState


class PandaProgramInterpreter(object):
    def __init__(self, robotless_debug=False):
        self.loaded_program = None
        self.next_primitive_index = -1  # next primitive to be executed!
        self.last_panda_status = None
        self.last_pose = None
        self.last_gripper_width = None
        self.last_gripper_force = None

        self.last_primitive_attempted = None
        # TODO: this is a quick HACK to let the widget verbalize the last primitive

        self.robotless_debug = robotless_debug
        self.fake_wait = 1

        self.revert_default_position_speed = .07  # m/s
        self.revert_default_rotation_speed = -1.0  # rad/s; see rotation_speed-less MoveTo** in primitive_interface.cpp

        # TO THE PRIMITIVE_INTERFACE, IF ANY
        if not self.robotless_debug:
            self.interface_state_subscriber = rospy.Subscriber("/primitive_interface_node/interface_state", Int32,
                                                               self.interface_state_callback)

            self.gripper_state_subscriber = rospy.Subscriber("/franka_gripper/joint_states", JointState,
                                                             self.gripper_state_callback)

            self.move_to_ee_client = actionlib.SimpleActionClient('/primitive_interface_node/move_to_ee_server',
                                                                  MoveToEEAction)
            self.move_to_contact_client = actionlib.SimpleActionClient(
                'primitive_interface_node/move_to_contact_server', MoveToContactAction)
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

    def gripper_state_callback(self, msg):
        last_gripper_width = msg.position[0] + msg.position[1]
        self.last_gripper_width = last_gripper_width if last_gripper_width <= 0.08 else 0.08

    def interface_state_callback(self, msg):
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
        self.next_primitive_index = 0
        success = self.go_to_current_primitive_preconditions(progress_callback)
        for i in range(1, self.loaded_program.get_program_length()):
            self.loaded_program.get_nth_primitive(i).status = pp.PandaPrimitiveStatus.NEUTRAL
        self.next_primitive_index = 0 if success else -1
        return success

    def go_to_current_primitive_preconditions(self, progress_callback=None):
        if self.loaded_program is None:
            rospy.logwarn('no program loaded')
            return False

        try:
            arm_state, gripper_state = self.loaded_program.get_nth_primitive_preconditions(self.next_primitive_index)
        except pp.PandaProgramException:
            # HACK for corner case
            # ouch, no preconditions -  we might be recovering from reverting the last primitive
            if self.next_primitive_index == self.loaded_program.get_program_length():
                # yes, we are - ok, let's just execute the last primitive
                self.next_primitive_index -= 1
                return self.execute_rest_of_program()
            else:
                rospy.logerr('Cannot find preconditions for the primitive: did you save them to begin with?')
                return False

        primitive_to_precon = self.loaded_program.get_nth_primitive(self.next_primitive_index)

        if not self.robotless_debug:
            # create new Goal for the primitive
            goal = MoveToEEGoal()
            goal.pose = arm_state
            goal.position_speed = self.revert_default_position_speed
            goal.rotation_speed = self.revert_default_rotation_speed

            primitive_to_precon.status = pp.PandaPrimitiveStatus.REVERTING

            if progress_callback is not None:
                progress_callback.emit(self.next_primitive_index)

            state = self.move_to_ee_client.send_goal_and_wait(goal, rospy.Duration(60))
            success_arm = (state == actionlib.GoalStatus.SUCCEEDED)
            result = self.move_to_ee_client.get_result()

            if success_arm:
                self.last_pose = result.final_pose

            rospy.loginfo('Success arm? :' + str(success_arm))

            # HACK:
            # Problem: requesting ApplyForceFingers when robot is already applying force locks the service caller
            # Solution: check if there was already force commanded (last_gripper_force > 0.0). If so, do nothing
            if gripper_state.force > 0.0:
                if self.last_gripper_force == 0.0:
                    # need to revert to a apply_force_fingers
                    request = ApplyForceFingersRequest()
                    request.force = gripper_state.force
                    try:
                        response = self.apply_force_fingers_client.call(request)
                    except rospy.ServiceException as e:
                        rospy.logerr('Error while contacting the apply_force_fingers server: {}'.format(e))
                        response = ApplyForceFingersResponse()
                        response.success = False
                    if response.success:
                        self.last_gripper_force = request.force
                else:
                    response = ApplyForceFingersResponse()
                    response.success = True
            else:
                # need to revert to a move_fingers
                request = MoveFingersRequest()
                request.width = gripper_state.width
                try:
                    response = self.move_fingers_client.call(request)
                except rospy.ServiceException as e:
                    rospy.logerr('Error while contacting the move_fingers server: {}'.format(e))
                    response = MoveFingersResponse()
                    response.success = False

            rospy.loginfo('Success gripper? :' + str(response.success))
        else:
            primitive_to_precon.status = pp.PandaPrimitiveStatus.REVERTING
            if progress_callback is not None:
                progress_callback.emit(self.next_primitive_index)
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True
            success_arm = True

        if success_arm and response.success:
            primitive_to_precon.status = pp.PandaPrimitiveStatus.READY
            try:
                if self.loaded_program.get_nth_primitive(self.next_primitive_index - 1).status == \
                        pp.PandaPrimitiveStatus.ERROR:
                    rospy.loginfo('Was resetting a Reversion error - set previous one to EXECUTED')
                    self.loaded_program.get_nth_primitive(self.next_primitive_index - 1).status = \
                        pp.PandaPrimitiveStatus.EXECUTED
                if progress_callback is not None:
                    progress_callback.emit(self.next_primitive_index)
            except pp.PandaProgramException:
                pass
        else:
            primitive_to_precon.status = pp.PandaPrimitiveStatus.ERROR
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

        self.last_primitive_attempted = primitive_to_execute  # TODO: this is a quick HACK
        callback = self.callback_switcher.get(primitive_to_execute.__class__, None)

        if callback is None:
            rospy.logwarn("I don't know how to execute this primitive; did you define the callback?")
            return False

        primitive_to_execute.status = pp.PandaPrimitiveStatus.EXECUTING
        if progress_callback is not None:
            progress_callback.emit(self.next_primitive_index)
        result = callback(primitive_to_execute)

        if result:
            primitive_to_execute.status = pp.PandaPrimitiveStatus.EXECUTED
            rospy.logdebug('Executed primitive ' + primitive_to_execute.__str__())
            self.next_primitive_index += 1
            try:
                next_primitive = self.loaded_program.get_nth_primitive(self.next_primitive_index)
                next_primitive.status = pp.PandaPrimitiveStatus.READY

                # TODO: maybe we should do this all the time, not only if the next primitive is not revertible,
                #  that is the current primitive was tuned at some point

                if not next_primitive.revertible:
                    self.loaded_program.update_nth_primitive_postconditions(
                        self.next_primitive_index - 1, [self.last_pose, pp.GripperState(self.last_gripper_width,
                                                                                        self.last_gripper_force)])

            except pp.PandaProgramException:
                pass
        else:
            primitive_to_execute.status = pp.PandaPrimitiveStatus.ERROR
            rospy.logerr('Error while executing ' + primitive_to_execute.__str__())

        return result

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

        self.last_primitive_attempted = None  # TODO: this is a quick HACK

        try:
            self.loaded_program.get_nth_primitive(self.next_primitive_index).status = \
                pp.PandaPrimitiveStatus.NEUTRAL
        except pp.PandaProgramException:
            pass

        callback = self.revert_callback_switcher.get(primitive_to_revert.__class__, None)

        if callback is None:
            rospy.logwarn("I don't know how to revert this primitive; did you define the revert callback?")
            return False

        primitive_to_revert.status = pp.PandaPrimitiveStatus.REVERTING
        if progress_callback is not None:
            progress_callback.emit(self.next_primitive_index)
        result = callback(self.next_primitive_index - 1)

        if result:
            primitive_to_revert.status = pp.PandaPrimitiveStatus.READY
            rospy.logdebug('Reverted primitive ' + primitive_to_revert.__str__())
            self.next_primitive_index -= 1
        else:
            primitive_to_revert.status = pp.PandaPrimitiveStatus.ERROR
            rospy.logerr('Error while reverting ' + primitive_to_revert.__str__())

        return result

    def execute_rest_of_program(self, one_shot_execution=False, progress_callback=None):
        partial_success = True
        to_be_restored_index = self.next_primitive_index

        if one_shot_execution:
            # TODO: this is just for the execute now (like relax_fingers and execute_primitive_now).
            #   Find a better way to do this
            self.next_primitive_index = 0

        primitive_counter = self.next_primitive_index

        while partial_success:
            partial_success = self.execute_one_step(progress_callback)
            if partial_success:
                primitive_counter += 1

        result = (primitive_counter == self.loaded_program.get_program_length())
        if result:
            rospy.loginfo('Executed rest of the program!')
        else:
            rospy.logerr('Something went south. Program now at step {}'.format(self.next_primitive_index))

        self.last_primitive_attempted = None  # TODO: this is a quick HACK

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
            self.go_to_starting_state()
            rospy.loginfo('Reverted to beginning of the program!')
            return True
        else:
            rospy.logerr('Something went south while reverting. Program now at step {}'.format(
                self.next_primitive_index))
            return False

    # PRIMITIVE CALLBACKS
    def execute_user_sync(self, primitive_to_execute):
        rospy.logdebug('Trying to execute a user sync')
        if not self.robotless_debug:
            state = self.user_sync_client.send_goal_and_wait(primitive_to_execute.parameter_container)
            success = (state == actionlib.GoalStatus.SUCCEEDED)
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            state = actionlib.GoalStatus.SUCCEEDED
            success = True

        rospy.loginfo('UserSync. S? {} [{}]'.format(str(success), str(state)))
        return success

    def execute_move_to_contact(self, primitive_to_execute):
        rospy.logdebug('Trying to execute a move to contact')

        if not self.robotless_debug:
            state = self.move_to_contact_client.send_goal_and_wait(primitive_to_execute.parameter_container,
                                                                   rospy.Duration(60))
            success = (state == actionlib.GoalStatus.SUCCEEDED)
            result = self.move_to_contact_client.get_result()

            if success:
                self.last_pose = result.final_pose
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            state = actionlib.GoalStatus.SUCCEEDED
            success = True

        rospy.loginfo('MoveToContact. S? {} [{}]'.format(str(success), str(state)))
        return success

    def execute_move_to_ee(self, primitive_to_execute):
        rospy.logdebug('Trying to execute a move to EE')

        if not self.robotless_debug:
            state = self.move_to_ee_client.send_goal_and_wait(primitive_to_execute.parameter_container,
                                                              rospy.Duration(60))
            success = (state == actionlib.GoalStatus.SUCCEEDED)
            result = self.move_to_ee_client.get_result()

            if success:
                self.last_pose = result.final_pose
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            state = actionlib.GoalStatus.SUCCEEDED
            success = True

        rospy.loginfo('MoveToEE. S? {} [{}]'.format(str(success), str(state)))
        return success

    def execute_move_fingers(self, primitive_to_execute):
        rospy.logdebug('Trying to execute a move fingers')
        if not self.robotless_debug:
            response = self.move_fingers_client.call(primitive_to_execute.parameter_container)

            if response.success:
                self.last_gripper_force = 0.0
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True
        rospy.loginfo('Move Fingers .S? :' + str(response.success))
        return response.success

    def execute_apply_force_fingers(self, primitive_to_execute):
        rospy.logdebug('Trying to execute an apply force with fingers')
        if not self.robotless_debug:
            response = self.apply_force_fingers_client.call(primitive_to_execute.parameter_container)

            if response.success:
                self.last_gripper_force = primitive_to_execute.parameter_container.force
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = ApplyForceFingersResponse()
            response.success = True
        rospy.loginfo('Apply Force. S? :' + str(response.success))
        return response.success

    # REVERT PRIMITIVE CALLBACK
    def revert_user_sync(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.logdebug('Trying to revert a user sync')

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        if not self.robotless_debug:
            state = self.move_to_ee_client.send_goal_and_wait(goal, rospy.Duration(60))
            success = (state == actionlib.GoalStatus.SUCCEEDED)
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            state = actionlib.GoalStatus.SUCCEEDED
            success = True

        rospy.loginfo('Revert UserSync. S? {} [{}]'.format(str(success), str(state)))
        return success

    def revert_move_to_contact(self, primitive_index):
        # TODO: reverting something AFTER a move_to_contact might not be that easy...
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.logdebug('Trying to revert a move to contact')

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        if not self.robotless_debug:
            state = self.move_to_ee_client.send_goal_and_wait(goal, rospy.Duration(60))
            success = (state == actionlib.GoalStatus.SUCCEEDED)
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            state = actionlib.GoalStatus.SUCCEEDED
            success = True

        rospy.loginfo('Revert MoveToContact. S? {} [{}]'.format(str(success), str(state)))
        return success

    def revert_move_to_ee(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.logdebug('Trying to revert a move to EE')

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = pose
        goal.position_speed = self.revert_default_position_speed
        goal.rotation_speed = self.revert_default_rotation_speed

        if not self.robotless_debug:
            state = self.move_to_ee_client.send_goal_and_wait(goal, rospy.Duration(60))
            success = (state == actionlib.GoalStatus.SUCCEEDED)
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            state = actionlib.GoalStatus.SUCCEEDED
            success = True

        rospy.loginfo('Revert MoveToEE. S? {} [{}]'.format(str(success), str(state)))
        return success

    def revert_move_fingers(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.logdebug('Trying to revert a move fingers to {}, {}'.format(gripper_state.width, gripper_state.force))

        if not self.robotless_debug:
            if gripper_state.force > 0.0:
                if self.last_gripper_force == 0.0:
                    # need to revert to a apply_force_fingers
                    request = ApplyForceFingersRequest()
                    request.force = gripper_state.force
                    try:
                        response = self.apply_force_fingers_client.call(request)
                    except rospy.ServiceException as e:
                        rospy.logerr('Error while contacting the apply_force_fingers server: {}'.format(e))
                        response = ApplyForceFingersResponse()
                        response.success = False
                    if response.success:
                        self.last_gripper_force = request.force
                else:
                    response = ApplyForceFingersResponse()
                    response.success = True
            else:
                # need to revert to a move_fingers
                request = MoveFingersRequest()
                request.width = gripper_state.width
                try:
                    response = self.move_fingers_client.call(request)
                except rospy.ServiceException as e:
                    rospy.logerr('Error while contacting the move_fingers server: {}'.format(e))
                    response = MoveFingersResponse()
                    response.success = False
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True

        rospy.loginfo('Revert Move Fingers. S? :' + str(response.success))
        return response.success

    def revert_apply_force_fingers(self, primitive_index):
        try:
            pose, gripper_state = self.loaded_program.get_nth_primitive_preconditions(primitive_index)
        except pp.PandaProgramException:
            rospy.logerr('Cannot revert: this primitive does not exist')
            return False

        rospy.logdebug('Trying to revert an apply force fingers to {}, {}'.format(gripper_state.width,
                                                                                 gripper_state.force))

        if not self.robotless_debug:
            if gripper_state.force > 0.0:
                if self.last_gripper_force == 0.0:
                    # need to revert to a apply_force_fingers
                    request = ApplyForceFingersRequest()
                    request.force = gripper_state.force
                    try:
                        response = self.apply_force_fingers_client.call(request)
                    except rospy.ServiceException as e:
                        rospy.logerr('Error while contacting the apply_force_fingers server: {}'.format(e))
                        response = ApplyForceFingersResponse()
                        response.success = False
                    if response.success:
                        self.last_gripper_force = request.force
                else:
                    response = ApplyForceFingersResponse()
                    response.success = True
            else:
                # need to revert to a move_fingers
                request = MoveFingersRequest()
                request.width = gripper_state.width
                try:
                    response = self.move_fingers_client.call(request)
                except rospy.ServiceException as e:
                    rospy.logerr('Error while contacting the move_fingers server: {}'.format(e))
                    response = MoveFingersResponse()
                    response.success = False
        else:
            rospy.sleep(rospy.Duration(self.fake_wait))
            response = MoveFingersResponse()
            response.success = True

        rospy.loginfo('Revert Apply Force. S? :' + str(response.success))
        return response.success
