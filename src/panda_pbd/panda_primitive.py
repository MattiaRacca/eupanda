#!/usr/bin/env python

from __future__ import division
import pickle
import os

from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest, MoveFingersRequest, ApplyForceFingersRequest


class PandaPrimitive(object):
    def __init__(self, description="An abstract Panda primitive"):
        self.description = description
        self.parameter_container = None  # either a Goal or a Request message, depending on the primitive
        self.expected_container = None  # type of the container expected, depending again on the primitive
        self.starting_arm_state_index = None  # pose of the robot at the beginning of this primitive
        self.starting_gripper_state_index = None  # gripper state at the beginning of this primitive
        self.parameters_with_effects_on_robot_state = None  # for the subclasses

    def __str__(self):
        return self.description + ' (' + str(self.starting_arm_state_index) + ', ' +\
               str(self.starting_gripper_state_index) + ')'

    def set_parameter_container(self, container):
        container_filled = isinstance(container, self.expected_container)
        if container_filled:
            self.parameter_container = container
        return container_filled

    def set_starting_conditions(self, starting_arm_state_index, starting_gripper_state_index):
        self.starting_arm_state_index = starting_arm_state_index
        self.starting_gripper_state_index = starting_gripper_state_index


class UserSync(PandaPrimitive):
    def __init__(self, description="A User Synchronization primitive"):
        super(UserSync, self).__init__(description)
        self.expected_container = UserSyncGoal
        self.parameters_with_effects_on_robot_state = []


class MoveToEE(PandaPrimitive):
    def __init__(self, description="A Move to End-Effector primitive"):
        super(MoveToEE, self).__init__(description)
        self.expected_container = MoveToEEGoal
        self.parameters_with_effects_on_robot_state = ['pose']


class MoveToContact(PandaPrimitive):
    def __init__(self, description="A Move to Contact primitive"):
        super(MoveToContact, self).__init__(description)
        self.expected_container = MoveToContactGoal
        self.parameters_with_effects_on_robot_state = ['pose', 'force_threshold', 'torque_threshold']


class MoveFingers(PandaPrimitive):
    def __init__(self, description="A Move Fingers primitive"):
        super(MoveFingers, self).__init__(description)
        self.expected_container = MoveFingersRequest
        self.parameters_with_effects_on_robot_state = ['width']


class ApplyForceFingers(PandaPrimitive):
    def __init__(self, description="A Apply Force (with) Fingers primitive"):
        super(ApplyForceFingers, self).__init__(description)
        self.expected_container = ApplyForceFingersRequest
        self.parameters_with_effects_on_robot_state = ['force']


class OpenGripper(PandaPrimitive):
    def __init__(self, description="A Open Gripper primitive"):
        super(OpenGripper, self).__init__(description)
        self.expected_container = OpenGripperRequest
        self.parameters_with_effects_on_robot_state = ['width']


class CloseGripper(PandaPrimitive):
    def __init__(self, description="A Close Gripper primitive"):
        super(CloseGripper, self).__init__(description)
        self.expected_container = CloseGripperRequest
        self.parameters_with_effects_on_robot_state = ['width', 'force']


class PandaProgram(object):
    def __init__(self, name="Unnamed Program", description="Empty Description"):
        self.name = name
        self.description = description
        self.primitives = []
        self.arm_state_list = []  # list of EE poses
        self.gripper_state_list = []  # list of GripperState

        # dictionary of the effect of primitives on the [arm_state_list, gripper_state_list]
        # e.g., MoveToEE affects the arm pose but not the gripper
        self.effect_of_primitive = {
            UserSync: [False, False],
            MoveToContact: [True, False],
            MoveToEE: [True, False],
            MoveFingers: [False, True],
            ApplyForceFingers: [False, True],
            OpenGripper: [False, True],
            CloseGripper: [False, True]
        }

        self.revertible = True  # to be set to False by SOME primitives update

    def __str__(self):
        full_description = self.name + ': ' + self.description + '\n'
        for i, prim in enumerate(self.primitives):
            full_description += str(i).zfill(3) + ': ' + prim.__str__() + '\n'

        return full_description

    def save_arm_state(self, arm_state):
        self.arm_state_list.append(arm_state)

    def save_gripper_state(self, gripper_state):
        self.gripper_state_list.append(gripper_state)

    def get_program_length(self):
        return len(self.primitives)

    def get_nth_primitive(self, n):
        if n < 0:  # stupid python with -n indexing
            raise PandaProgramException(0)
        try:
            return self.primitives[n]
        except IndexError:
            raise PandaProgramException(0)

    def get_nth_primitive_preconditions(self, n):
        try:
            prim = self.get_nth_primitive(n)
        except PandaProgramException:
            return None, None  # TODO: fix accordingly

        arm_state = self.arm_state_list[prim.starting_arm_state_index]
        gripper_state = self.gripper_state_list[prim.starting_gripper_state_index]
        return arm_state, gripper_state

    def get_nth_primitive_postcondition_indexes(self, n):
        try:
            next_primitive = self.get_nth_primitive(n + 1)
        except PandaProgramException:
            if n == self.get_program_length() - 1:
                next_primitive = None
            else:
                raise PandaProgramException(0)  # TODO: better handling here?

        if next_primitive is None:  # N is the last primitive
            return -1, -1
        else:
            return next_primitive.starting_arm_state_index, next_primitive.starting_gripper_state_index

    def insert_primitive(self, primitive, post_conditions):
        self.primitives.append(primitive)

        self.primitives[-1].starting_arm_state_index = len(self.arm_state_list) - 1
        self.primitives[-1].starting_gripper_state_index = len(self.gripper_state_list) - 1

        effect = self.effect_of_primitive.get(primitive.__class__, [False, False])

        if effect[0]:
            self.save_arm_state(post_conditions[0])
        if effect[1]:
            self.save_gripper_state(post_conditions[1])

        return len(self.primitives) - 1

    def delete_nth_primitive(self, n):
        try:
            to_be_deleted = self.get_nth_primitive(n)
        except PandaProgramException:
            return False

        try:
            arm_state_index, gripper_state_index = self.get_nth_primitive_postcondition_indexes(n)
        except PandaProgramException:
            return False

        effect = self.effect_of_primitive.get(to_be_deleted.__class__, [False, False])

        if effect[0]:
            del self.arm_state_list[arm_state_index]
        if effect[1]:
            del self.gripper_state_list[gripper_state_index]

        for primitive in self.primitives[n+1:len(self.primitives)]:
            if effect[0] and primitive.starting_arm_state_index == arm_state_index:
                primitive.starting_arm_state_index -= 1
            if effect[1] and primitive.starting_gripper_state_index == gripper_state_index:
                primitive.starting_gripper_state_index -= 1

        del self.primitives[n]
        return True

    def update_nth_primitive_parameters(self, n, parameter_name, parameter_value):
        try:
            primitive = self.get_nth_primitive(n)
        except PandaProgramException:
            return False  # TODO: better handling?

        if hasattr(primitive.expected_container, parameter_name) is False:
            return False  # no parameter with name parameter_name  TODO: raise error instead?

        try:
            primitive.parameters_with_effects_on_robot_state.index(parameter_name)
            revertible = False
        except ValueError:
            revertible = True

        setattr(primitive.parameter_container, parameter_name, parameter_value)
        if not revertible:
            self.revertible = False

        return True

    def update_nth_primitive_postconditions(self, n, new_post_conditions):
        try:
            primitive = self.get_nth_primitive(n)
        except PandaProgramException:
            return False  # TODO: better handling?
        try:
            arm_index, gripper_index = self.get_nth_primitive_postcondition_indexes(n)
        except PandaProgramException:
            return False  # TODO: better handling?

        effect = self.effect_of_primitive.get(primitive.__class__, [False, False])

        if effect[0]:
            self.arm_state_list[arm_index] = new_post_conditions[0]
        if effect[1]:
            self.gripper_state_list[gripper_index] = new_post_conditions[1]

        self.revertible = True  # TODO: probably better to move revertible inside each Primitive!
        return True

    def dump_to_file(self, filepath='~', filename='program.pkl'):
        dump_program_to_file(self, filepath, filename)


class PandaProgramException(Exception):
    def __init__(self, error_id):
        error_messages = {
            0: 'Error: Primitive nth does not exist'
        }
        message = error_messages.get(error_id, 'Unknown Error')
        super(PandaProgramException, self).__init__(message)


class GripperState(object):
    def __init__(self, width, force):
        self.width = width
        self.force = force


def dump_program_to_file(program, filepath='~', filename='program.pkl'):
    with open(os.path.join(os.path.expanduser(filepath), filename), 'wb') as f:
        pickle.dump(program, f)


def load_program_from_file(filepath='~', filename='program.pkl'):
    with open(os.path.join(os.path.expanduser(filepath), filename), 'rb') as f:
        loaded_program = pickle.load(f)
        return loaded_program
