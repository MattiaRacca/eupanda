#!/usr/bin/env python

import rospy
import panda_primitive as pp
from panda_pbd.srv import EnableTeaching, EnableTeachingRequest
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest
from sensor_msgs.msg import JointState

class PandaProgrammingByDemonstrationInterface(object):
    def __init__(self):
        self.program = pp.PandaProgram('A Panda Program')
        self.ft_threshold = 5.0

        # TODO: this implementation of last pose is probably not the best way to do this
        self.last_pose = None
        self.last_gripper_width = None

        try:
            self.gripper_state_subscriber  = rospy.Subscriber("/franka_gripper/joint_states", \
                                                              JointState, self.gripper_state_callback)
        except:
            rospy.logerr('Something went from with the gripper state subscriber')

        # TODO: make the default values encoded in ros parameters
        self.move_to_ee_default_position_speed = .04 # m/s
        self.move_to_ee_default_rotation_speed = 1.0  # rad/s

        self.user_sync_default_force_threshold = 10.0 # N

        self.close_gripper_default_force = 5.0 # N

        self.move_to_contact_default_force_threshold = 5.0 # N
        self.move_to_contact_default_torque_threshold = 5.0 # Nm/rad
        self.move_to_contact_default_position_speed = .04 # m/s
        self.move_to_contact_default_rotation_speed = 1.0  # rad/s

        try:
            rospy.wait_for_service('/primitive_interface_node/kinesthetic_teaching', 20)
        except rospy.ROSException:
            rospy.logerr('Cannot contact the Primitive Interface Node!')

        try:
            self.kinesthetic_client = rospy.ServiceProxy('/primitive_interface_node/kinesthetic_teaching',\
                                                         EnableTeaching)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')

    def gripper_state_callback(self, data):
        # TODO: self.last_gripper_width = data something?
        pass

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

    def insert_move_to_ee(self):
        if self.last_pose is None:
            return False
        else:
            goal = MoveToEEGoal()
            goal.pose = self.last_pose
            goal.position_speed = self.move_to_ee_default_position_speed
            goal.rotation_speed = self.move_to_ee_default_rotation_speed

            move_to_ee_primitive = pp.MoveToEE()
            move_to_ee_primitive.set_parameter_container(goal)
            self.program.insert_primitive(move_to_ee_primitive)

    def insert_move_to_contact(self):
        if self.last_pose is None:
            return False
        else:
            goal = MoveToContactGoal()
            goal.pose = self.last_pose
            goal.position_speed = self.move_to_contact_default_position_speed
            goal.rotation_speed = self.move_to_contact_default_rotation_speed
            goal.force_threshold = self.move_to_contact_default_force_threshold
            goal.torque_threshold = self.move_to_contact_default_torque_threshold

            move_to_contact_primitive = pp.MoveToContact()
            move_to_contact_primitive.set_parameter_container(goal)
            self.program.insert_primitive(move_to_contact_primitive)

    def insert_user_sync(self):
        goal = UserSyncGoal()
        goal.force_threshold = self.user_sync_default_force_threshold

        user_sync_primitive = pp.UserSync()
        self.program.insert_primitive(user_sync_primitive)

    def insert_close_gripper(self):
        request = CloseGripperRequest()
        request.width = self.last_gripper_width
        request.force = self.close_gripper_default_force

        close_gripper_primitive = pp.CloseGripper()
        close_gripper_primitive.set_parameter_container(request)
        self.program.insert_primitive(close_gripper_primitive)

    def insert_open_gripper(self):
        request = OpenGripperRequest()
        request.width = self.last_gripper_width

        open_gripper_primitive = pp.OpenGripper()
        open_gripper_primitive.set_parameter_container(request)
        self.program.insert_primitive(open_gripper_primitive)
