#!/usr/bin/env python

from __future__ import division

class PandaPrimitive(object):
    def __init__(self, id=None, description="An abstract Panda primitive"):
        self.id = id
        self.description = description
        self.parameters = {}
        self.fully_specified = False

    def __str__(self):
        return "Primitive: " + self.id + ": " + self.description

    def set_all_parameters(self, **kwargs):
        self.fully_specified = all(param in kwargs.keys() for param in self.parameters.keys())
        if self.fully_specified:
            for param in kwargs.keys():
                self.parameters[param] = kwargs[param]
        return self.fully_specified

class UserSync(PandaPrimitive):
    def __init__(self, id=None, description="A User Synchronization primitive"):
        super(PandaPrimitive, self).__init__(id, description)
        self.parameters = {'force_threshold': None}

class MoveToEE(PandaPrimitive):
    def __init__(self, id=None, description="A Move to End-Effector primitive"):
        super(PandaPrimitive, self).__init__(id, description)
        self.parameters = {'pose': None,\
                           'position_speed': None,\
                           'rotation_speed': None}

class MoveToContact(PandaPrimitive):
    def __init__(self, id=None, description="A Move to Contact primitive"):
        super(PandaPrimitive, self).__init__(id, description)
        self.parameters = {'pose': None, \
                           'position_speed': None, \
                           'rotation_speed': None, \
                           'force_threshold': None, \
                           'torque_threshold': None}

class OpenGripper(PandaPrimitive):
    def __init__(self, id=None, description="A Open Gripper primitive"):
        super(PandaPrimitive, self).__init__(id, description)
        self.parameters = {'width': None}

class CloseGripper(PandaPrimitive):
    def __init__(self, id=None, description="A Close Gripper primitive"):
        super(PandaPrimitive, self).__init__(id, description)
        self.parameters = {'width': None,\
                           'force': None}
