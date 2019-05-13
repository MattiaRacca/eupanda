#!/usr/bin/env python

from __future__ import division
from collections import OrderedDict

# TODO: maybe instead of self.parameters could I put directly the msg or the service req that I need?

class PandaPrimitive(object):
    def __init__(self, description="An abstract Panda primitive"):
        self.description = description
        self.parameters = {}
        self.fully_specified = False

    def __str__(self):
        return self.description

    def set_all_parameters(self, **kwargs):
        self.fully_specified = all(param in kwargs.keys() for param in self.parameters.keys())
        if self.fully_specified:
            for param in kwargs.keys():
                self.parameters[param] = kwargs[param]
        return self.fully_specified

class UserSync(PandaPrimitive):
    def __init__(self, description="A User Synchronization primitive"):
        super(UserSync, self).__init__(description)
        self.parameters = {'force_threshold': None}

class MoveToEE(PandaPrimitive):
    def __init__(self, description="A Move to End-Effector primitive"):
        super(MoveToEE, self).__init__(description)
        self.parameters = {'pose': None,\
                           'position_speed': None,\
                           'rotation_speed': None}

class MoveToContact(PandaPrimitive):
    def __init__(self, description="A Move to Contact primitive"):
        super(MoveToContact, self).__init__(description)
        self.parameters = {'pose': None, \
                           'position_speed': None, \
                           'rotation_speed': None, \
                           'force_threshold': None, \
                           'torque_threshold': None}

class OpenGripper(PandaPrimitive):
    def __init__(self, description="A Open Gripper primitive"):
        super(OpenGripper, self).__init__(description)
        self.parameters = {'width': None}

class CloseGripper(PandaPrimitive):
    def __init__(self, description="A Close Gripper primitive"):
        super(CloseGripper, self).__init__(description)
        self.parameters = {'width': None,\
                           'force': None}

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
        return self.name + ': ' + self.description

    def delete_primitive(self, id):
        try:
            del self.primitives[id]
            return True
        except:
            return False
