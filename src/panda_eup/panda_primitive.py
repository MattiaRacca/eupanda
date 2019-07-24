#!/usr/bin/env python

from __future__ import division
import pickle
import os
from enum import Enum

from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest, MoveFingersRequest, ApplyForceFingersRequest


class PandaPrimitiveStatus(Enum):
    NEUTRAL = 0
    EXECUTED = 1
    EXECUTING = 2
    REVERTING = 3
    ERROR = 4
    READY = 5


class PandaRobotStatus(Enum):
    ERROR = 0
    READY = 1
    BUSY = 2


class PandaPrimitive(object):
    gui_tunable_parameters = None  # for the subclasses
    gui_tunable_parameter_ranges = None  # for the subclasses
    gui_tunable_parameter_units = None  # for the subclasses

    def __init__(self, description="An abstract Panda primitive"):
        self.description = description
        self.parameter_container = None  # either a Goal or a Request message, depending on the primitive
        self.expected_container = None  # type of the container expected, depending again on the primitive
        self.starting_arm_state_index = None  # pose of the robot at the beginning of this primitive
        self.starting_gripper_state_index = None  # gripper state at the beginning of this primitive
        self.parameters_with_effects_on_robot_state = None  # for the subclasses
        self.revertible = True
        self.status = PandaPrimitiveStatus.NEUTRAL

    def __str__(self):
        return self.description + ' (' + str(self.starting_arm_state_index) + ', ' +\
               str(self.starting_gripper_state_index) + ' R: ' + str(self.revertible) + ' S: ' + self.status.name + ')'

    def set_parameter_container(self, container):
        container_filled = isinstance(container, self.expected_container)
        if container_filled:
            self.parameter_container = container
        return container_filled

    def set_starting_conditions(self, starting_arm_state_index, starting_gripper_state_index):
        self.starting_arm_state_index = starting_arm_state_index
        self.starting_gripper_state_index = starting_gripper_state_index

    def update_parameter(self, parameter_type, parameter_value):
        if hasattr(self.expected_container, parameter_type) is False:
            raise PandaProgramException(1)

        try:
            self.parameters_with_effects_on_robot_state.index(parameter_type)
            revertible = False
        except ValueError:
            revertible = True

        setattr(self.parameter_container, parameter_type, parameter_value)
        return revertible


class UserSync(PandaPrimitive):
    gui_tunable_parameters = ['force_threshold']
    gui_tunable_parameter_ranges = {
        gui_tunable_parameters[0]: [0.0, 10.0]
    }
    gui_tunable_parameter_units = {
        gui_tunable_parameters[0]: 'N'
    }

    def __init__(self, description="A User Synchronization primitive"):
        super(UserSync, self).__init__(description)
        self.expected_container = UserSyncGoal
        self.parameters_with_effects_on_robot_state = []


class MoveToEE(PandaPrimitive):
    gui_tunable_parameters = ['position_speed']
    gui_tunable_parameter_ranges = {
        gui_tunable_parameters[0]: [0.001, 0.5]
    }
    gui_tunable_parameter_units = {
        gui_tunable_parameters[0]: 'm/s'
    }

    def __init__(self, description="A Move to End-Effector primitive"):
        super(MoveToEE, self).__init__(description)
        self.expected_container = MoveToEEGoal
        self.parameters_with_effects_on_robot_state = ['pose']


class MoveToContact(PandaPrimitive):
    gui_tunable_parameters = ['position_speed', 'force_threshold']
    gui_tunable_parameter_ranges = {
        gui_tunable_parameters[0]: [0.001, 0.5],
        gui_tunable_parameters[1]: [0.0, 10.0]
    }
    gui_tunable_parameter_units = {
        gui_tunable_parameters[0]: 'm/s',
        gui_tunable_parameters[1]: 'N'
    }

    def __init__(self, description="A Move to Contact primitive"):
        super(MoveToContact, self).__init__(description)
        self.expected_container = MoveToContactGoal
        self.parameters_with_effects_on_robot_state = ['pose', 'force_threshold', 'torque_threshold']


class MoveFingers(PandaPrimitive):
    gui_tunable_parameters = ['width']
    gui_tunable_parameter_ranges = {
        gui_tunable_parameters[0]: [0.0, 0.08]
    }
    gui_tunable_parameter_units = {
        gui_tunable_parameters[0]: 'm'
    }

    def __init__(self, description="A Move Fingers primitive"):
        super(MoveFingers, self).__init__(description)
        self.expected_container = MoveFingersRequest
        self.parameters_with_effects_on_robot_state = ['width']


class ApplyForceFingers(PandaPrimitive):
    gui_tunable_parameters = ['force']
    gui_tunable_parameter_ranges = {
        gui_tunable_parameters[0]: [0.0, 10.0]
    }
    gui_tunable_parameter_units = {
        gui_tunable_parameters[0]: 'N'
    }

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
            raise

        if prim.starting_arm_state_index < 0 or prim.starting_gripper_state_index < 0:
            raise PandaProgramException(0)

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
                raise

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
            raise

        try:
            arm_state_index, gripper_state_index = self.get_nth_primitive_postcondition_indexes(n)
        except PandaProgramException:
            raise

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

    def update_nth_primitive_parameter(self, n, parameter_name, parameter_value):
        try:
            primitive = self.get_nth_primitive(n)
        except PandaProgramException:
            raise

        try:
            revertible = primitive.update_parameter(parameter_name, parameter_value)
            tuned = True
        except PandaProgramException:
            revertible = True
            tuned = False

        try:
            next_primitive = self.get_nth_primitive(n + 1)
        except PandaProgramException:
            pass
        else:
            next_primitive.revertible = revertible

        return tuned

    def update_nth_primitive_postconditions(self, n, new_post_conditions):
        try:
            primitive = self.get_nth_primitive(n)
        except PandaProgramException:
            raise
        try:
            arm_index, gripper_index = self.get_nth_primitive_postcondition_indexes(n)
        except PandaProgramException:
            raise

        effect = self.effect_of_primitive.get(primitive.__class__, [False, False])

        if effect[0]:
            self.arm_state_list[arm_index] = new_post_conditions[0]
        if effect[1]:
            self.gripper_state_list[gripper_index] = new_post_conditions[1]

        try:
            next_primitive = self.get_nth_primitive(n + 1)
        except PandaProgramException:
            pass
        else:
            next_primitive.revertible = True

        return True

    def dump_to_file(self, filepath='~', filename='program.pkl'):
        dump_program_to_file(self, filepath, filename)


class PandaProgramException(Exception):
    def __init__(self, error_id):
        error_messages = {
            0: 'Error: Primitive nth does not exist',
            1: 'Error: Wrong parameter name in the primitive update'
        }
        message = error_messages.get(error_id, 'Unknown Error')
        super(PandaProgramException, self).__init__(message)


class GripperState(object):
    def __init__(self, width, force):
        self.width = width
        self.force = force

    def __str__(self):
        return 'Gripper state. Width: {}, Force: {}'.format(self.width, self.force)


def dump_program_to_file(program, filepath='~', filename='program.pkl'):
    with open(os.path.join(os.path.expanduser(filepath), filename), 'wb') as f:
        pickle.dump(program, f)


def load_program_from_file(filepath='~', filename='program.pkl'):
    with open(os.path.join(os.path.expanduser(filepath), filename), 'rb') as f:
        loaded_program = pickle.load(f)
        return loaded_program
