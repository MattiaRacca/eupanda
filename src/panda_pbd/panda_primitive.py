#!/usr/bin/env python

from __future__ import division
from collections import OrderedDict
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest

class PandaPrimitive(object):
    def __init__(self, description="An abstract Panda primitive"):
        self.description = description
        self.parameter_container = None # either a Goal or a Request message, depending on the primitive
        self.expected_container = None # type of the container expected, depending again on the primitive

    def __str__(self):
        return self.description

    def set_parameter_container(self, container):
        container_filled = isinstance(container, self.expected_container)
        if container_filled:
            self.parameter_container = container
        return container_filled

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

    def insert_primitive(self, primitive):
        if len(self.primitives.items()) == 0:
            id = 0
            self.primitives[0] = primitive
        else:
            id = max(self.primitives.keys()) + 1
            self.primitives[id] = primitive
        return id

    def __str__(self):
        full_description = ''
        full_description += self.name + ': ' + self.description + '\n'
        # TODO: iterate over the program dictionary
        # for prim in self.primitives:
        #    full_description +=

        return full_description

    def delete_primitive(self, id):
        try:
            del self.primitives[id]
            return True
        except:
            return False
