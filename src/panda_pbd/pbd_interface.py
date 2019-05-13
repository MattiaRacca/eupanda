#!/usr/bin/env python

import rospy
import panda_primitive as primitive
from panda_pbd.srv import EnableTeaching, EnableTeachingRequest, EnableTeachingResponse

class ProgrammingByDemonstrationInterface(object):
    def __init__(self):
        self.program = primitive.PandaProgram('A Panda Program')
        self.ft_threshold = 5.0
        self.last_pose = None

        try:
            rospy.wait_for_service('/primitive_interface_node/kinesthetic_teaching', 20)
        except rospy.ROSException:
            rospy.logerr('Cannot contact the Primitive Interface Node!')

        try:
            self.kinesthetic_client = rospy.ServiceProxy('/primitive_interface_node/kinesthetic_teaching',\
                                                         EnableTeaching)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')

    def relax(self):
        req = EnableTeachingRequest
        req.ft_threshold_multiplier = self.ft_threshold
        req.teaching = 1

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            return False

        if res.success:
            self.last_pose = res.ee_pose
        return True

    def relax_only_arm(self):
        req = EnableTeachingRequest
        req.ft_threshold_multiplier = self.ft_threshold
        req.teaching = 2

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            return False

        if res.success:
            self.last_pose = res.ee_pose
        return True

    def relax_only_wrist(self):
        req = EnableTeachingRequest
        req.ft_threshold_multiplier = self.ft_threshold
        req.teaching = 3

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            return False

        if res.success:
            self.last_pose = res.ee_pose
        return True

    def freeze(self):
        req = EnableTeachingRequest
        req.ft_threshold_multiplier = self.ft_threshold
        req.teaching = 0

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            return False

        if res.success:
            self.last_pose = res.ee_pose
        return True