#!/usr/bin/env python

import rospy
import copy
import panda_primitive as pp
import program_interpreter as interpreter
from panda_pbd.srv import EnableTeaching, EnableTeachingRequest
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import CloseGripperRequest, OpenGripperRequest
from sensor_msgs.msg import JointState


class PandaPBDInterface(object):
    def __init__(self):
        self.program = pp.PandaProgram('A Panda Program')

        self.last_pose = None
        self.last_gripper_width = None
        self.relaxed = False

        # TODO: make the default values encoded in ros parameters
        self.kinesthestic_ft_threshold = 5.0

        self.move_to_ee_default_position_speed = .04  # m/s
        self.move_to_ee_default_rotation_speed = 1.0  # rad/s

        self.user_sync_default_force_threshold = 10.0  # N

        self.close_gripper_default_force = 5.0  # N

        self.move_to_contact_default_force_threshold = 5.0  # N
        self.move_to_contact_default_torque_threshold = 5.0  # Nm/rad
        self.move_to_contact_default_position_speed = .04  # m/s
        self.move_to_contact_default_rotation_speed = 1.0  # rad/s

        self.gripper_state_subscriber = rospy.Subscriber("/franka_gripper/joint_states", JointState,
                                                         self.gripper_state_callback)

        try:
            self.kinesthetic_client = rospy.ServiceProxy('/primitive_interface_node/kinesthetic_teaching',
                                                         EnableTeaching)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')

        try:
            self.kinesthetic_client.wait_for_service(5.0)  # TODO: why 5 seconds?
        except rospy.ROSException:
            rospy.logerr('Cannot contact the Primitive Interface Node!')

        self.interpreter = interpreter.PandaProgramInterpreter()  # internal interpreter, for now actions!

        self.freeze()

        while self.last_pose is None or self.last_gripper_width is None:
            rospy.sleep(1.0)

        self.program.save_arm_state(self.last_pose)
        self.program.save_gripper_state(self.last_gripper_width)

    def gripper_state_callback(self, data):
        self.last_gripper_width = data.position[0] + data.position[1]

    def relax(self):
        req = EnableTeachingRequest()
        req.ft_threshold_multiplier = self.kinesthestic_ft_threshold
        req.teaching = 1

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            self.last_pose = None
            return False

        if res.success:
            self.last_pose = res.ee_pose
            self.relaxed = True
        return True

    def relax_only_arm(self):
        req = EnableTeachingRequest()
        req.ft_threshold_multiplier = self.kinesthestic_ft_threshold
        req.teaching = 2

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            self.last_pose = None
            return False

        if res.success:
            self.last_pose = res.ee_pose
            self.relaxed = True
        return True

    def relax_only_wrist(self):
        req = EnableTeachingRequest()
        req.ft_threshold_multiplier = self.kinesthestic_ft_threshold
        req.teaching = 3

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            self.last_pose = None
            return False

        if res.success:
            self.last_pose = res.ee_pose
            self.relaxed = True
        return True

    def relax_finger(self):
        # TODO: to be tested
        temp_program = pp.PandaProgram()
        request = OpenGripperRequest()
        request.width = self.last_gripper_width

        open_gripper_primitive = pp.OpenGripper()
        open_gripper_primitive.set_parameter_container(request)

        temp_program.insert_primitive(open_gripper_primitive, [None, self.last_gripper_width])

        self.interpreter.load_program(temp_program)
        return self.interpreter.execute_rest_of_program()

    def freeze(self):
        req = EnableTeachingRequest()
        req.teaching = 0

        try:
            res = self.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot create Kinesthetic Teaching client!')
            self.last_pose = None
            return False

        if res.success:
            self.last_pose = res.ee_pose
            self.relaxed = False
        return True

    def insert_move_to_ee(self):
        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = self.last_pose
        goal.position_speed = self.move_to_ee_default_position_speed
        goal.rotation_speed = self.move_to_ee_default_rotation_speed

        # create the primitive to be added
        move_to_ee_primitive = pp.MoveToEE()
        move_to_ee_primitive.set_parameter_container(goal)
        self.program.insert_primitive(move_to_ee_primitive, [goal.pose, None])

        if was_relaxed:
            self.relax()

    def insert_move_to_contact(self):
        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        goal = MoveToContactGoal()
        goal.pose = self.last_pose
        goal.position_speed = self.move_to_contact_default_position_speed
        goal.rotation_speed = self.move_to_contact_default_rotation_speed
        goal.force_threshold = self.move_to_contact_default_force_threshold
        goal.torque_threshold = self.move_to_contact_default_torque_threshold

        move_to_contact_primitive = pp.MoveToContact()
        move_to_contact_primitive.set_parameter_container(goal)
        self.program.insert_primitive(move_to_contact_primitive, [goal.pose, None])

        if was_relaxed:
            self.relax()

    def insert_user_sync(self):
        goal = UserSyncGoal()
        goal.force_threshold = self.user_sync_default_force_threshold

        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        user_sync_primitive = pp.UserSync()
        self.program.insert_primitive(user_sync_primitive, [None, None])

        if was_relaxed:
            self.relax()

    def insert_close_gripper(self):
        request = CloseGripperRequest()
        request.width = self.last_gripper_width
        request.force = self.close_gripper_default_force

        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        close_gripper_primitive = pp.CloseGripper()
        close_gripper_primitive.set_parameter_container(request)

        # TODO: this is probably not enough to revert!
        self.program.insert_primitive(close_gripper_primitive, [None, request.width])

        # TODO: is it worth to investigate this copy bug? with the deepcopy the code works as intended...
        self.execute_primitive_now(copy.deepcopy(close_gripper_primitive))

        if was_relaxed:
            self.relax()

    def insert_open_gripper(self):
        request = OpenGripperRequest()
        request.width = self.last_gripper_width

        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        open_gripper_primitive = pp.OpenGripper()
        open_gripper_primitive.set_parameter_container(request)
        self.program.insert_primitive(open_gripper_primitive, [None, request.width])

        # TODO: is it worth to investigate this copy bug? with the deepcopy the code works as intended...
        self.execute_primitive_now(copy.deepcopy(open_gripper_primitive))

        if was_relaxed:
            self.relax()

    def execute_primitive_now(self, primitive):
        temp_program = pp.PandaProgram()
        temp_program.insert_primitive(primitive, [self.last_pose, self.last_gripper_width])

        self.interpreter.load_program(temp_program)
        return self.interpreter.execute_rest_of_program()

    def reset_program(self):
        pass
