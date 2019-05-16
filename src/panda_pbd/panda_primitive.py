#!/usr/bin/env python

from __future__ import division
from collections import OrderedDict
import pickle
import os

from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest


class PandaPrimitive(object):
    def __init__(self, description="An abstract Panda primitive"):
        self.description = description
        self.parameter_container = None  # either a Goal or a Request message, depending on the primitive
        self.expected_container = None  # type of the container expected, depending again on the primitive
        self.starting_arm_state_id = None  # pose of the robot at the beginning of this primitive
        self.starting_gripper_state_id = None  # gripper state at the beginning of this primitive (finger width)

    def __str__(self):
        return self.description + '(' + self.starting_arm_state_id + ', ' + self.starting_gripper_state_id + ')'

    def set_parameter_container(self, container):
        container_filled = isinstance(container, self.expected_container)
        if container_filled:
            self.parameter_container = container
        return container_filled

    def set_starting_conditions(self, starting_arm_state_id, starting_gripper_state_id):
        self.starting_arm_state_id = starting_arm_state_id
        self.starting_gripper_state_id = starting_gripper_state_id


class UserSync(PandaPrimitive):
    def __init__(self, description="A User Synchronization primitive"):
        super(UserSync, self).__init__(description)
        self.expected_container = UserSyncGoal


class MoveToEE(PandaPrimitive):
    def __init__(self, description="A Move to End-Effector primitive"):
        super(MoveToEE, self).__init__(description)
        self.expected_container = MoveToEEGoal


class MoveToContact(PandaPrimitive):
    def __init__(self, description="A Move to Contact primitive"):
        super(MoveToContact, self).__init__(description)
        self.expected_container = MoveToContactGoal


class OpenGripper(PandaPrimitive):
    def __init__(self, description="A Open Gripper primitive"):
        super(OpenGripper, self).__init__(description)
        self.expected_container = OpenGripperRequest


class CloseGripper(PandaPrimitive):
    def __init__(self, description="A Close Gripper primitive"):
        super(CloseGripper, self).__init__(description)
        self.expected_container = CloseGripperRequest


class PandaProgram(object):
    def __init__(self, name="Unnamed Program", description="Empty Description"):
        self.name = name
        self.description = description
        self.primitives = OrderedDict()
        self.arm_state_list = OrderedDict()
        self.gripper_state_list = OrderedDict()

    def save_arm_state(self, arm_state):
        if len(self.arm_state_list.items()) == 0:
            id = 0
            self.arm_state_list[0] = arm_state
        else:
            id = max(self.arm_state_list.keys()) + 1
            self.arm_state_list[id] = arm_state
        return id

    def save_gripper_state(self, gripper_state):
        if len(self.gripper_state_list.items()) == 0:
            id = 0
            self.gripper_state_list[0] = gripper_state
        else:
            id = max(self.gripper_state_list.keys()) + 1
            self.gripper_state_list[id] = gripper_state
        return id

    def get_nth_primitive_preconditions(self, N):
        arm_state = self.arm_state_list[self.get_nth_primitive(N).starting_arm_state_id]
        gripper_state = self.gripper_state_list[self.get_nth_primitive(N).starting_gripper_state_id]
        return arm_state, gripper_state

    def insert_primitive(self, primitive):
        if len(self.primitives.items()) == 0:
            id = 0
            self.primitives[0] = primitive
        else:
            id = max(self.primitives.keys()) + 1
            self.primitives[id] = primitive

        self.primitives[id].starting_arm_state_id = self.arm_state_list.keys()[-1]
        self.primitives[id].starting_arm_state_id = self.arm_state_list.keys()[-1]
        return id

    def __str__(self):
        full_description = ''
        full_description += self.name + ': ' + self.description + '\n'
        for prim in self.primitives.items():
            full_description += str(prim[0]).zfill(3) + ': ' + prim[1].__str__() + '\n'

        return full_description

    def delete_primitive(self, id):
        try:
            del self.primitives[id]
            return True
        except:
            return False

    def get_program_length(self):
        return len(self.primitives.values())

    def get_nth_primitive(self, N):
        try:
            return self.primitives.values()[N]
        except:
            return None

    def dump_to_file(self, filepath='~', filename='program.pkl'):
        dump_program_to_file(self, filepath, filename)


def dump_program_to_file(program, filepath='~', filename='program.pkl'):
    with open(os.path.join(os.path.expanduser(filepath), filename), 'wb') as f:
        pickle.dump(program, f)


def load_program_from_file(filepath='~', filename='program.pkl'):
    with open(os.path.join(os.path.expanduser(filepath), filename), 'rb') as f:
        loaded_program = pickle.load(f)
        return loaded_program
