#!/usr/bin/env python

import rospy
import copy
import numpy as np
import panda_primitive as pp
import program_interpreter as interpreter
from panda_pbd.srv import EnableTeaching, EnableTeachingRequest
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import MoveFingersRequest, ApplyForceFingersRequest
from sensor_msgs.msg import JointState


class PandaPBDInterface(object):
    def __init__(self, robotless_debug):
        self.program = pp.PandaProgram('A Panda Program')
        self.robotless_debug = robotless_debug
        self.last_pose = None
        self.last_gripper_width = None
        self.relaxed = False

        self.default_parameters = {'kinesthestic_ft_threshold': 5.0,
                                   'move_to_ee_default_position_speed': 0.07,
                                   'move_to_ee_default_rotation_speed': -1.0,
                                   'user_sync_default_force_threshold': 10.0,
                                   'apply_force_fingers_default_force': 20.0,
                                   'move_to_contact_default_force_threshold': 10.0,
                                   'move_to_contact_default_torque_threshold': 10.0,
                                   'move_to_contact_default_position_speed': 0.07,
                                   'move_to_contact_default_rotation_speed': -1.0}

        for parameter_name in self.default_parameters.keys():
            if not rospy.has_param('~' + parameter_name):
                rospy.logwarn('Parameter ' + parameter_name + ' not found. Will use default parameter {}'.
                              format(self.default_parameters[parameter_name]))
            else:
                self.default_parameters[parameter_name] = rospy.get_param('~' + parameter_name,
                                                                          self.default_parameters[parameter_name])
        if not self.robotless_debug:
            self.gripper_state_subscriber = rospy.Subscriber("/franka_gripper/joint_states", JointState,
                                                            self.gripper_state_callback)

            try:
                self.kinesthetic_client = rospy.ServiceProxy('/primitive_interface_node/kinesthetic_teaching',
                                                            EnableTeaching)
            except rospy.ServiceException:
                rospy.logerr('Cannot create Kinesthetic Teaching client!')

            try:
                self.kinesthetic_client.wait_for_service(5.0)
            except rospy.ROSException:
                rospy.logerr('Cannot contact the Primitive Interface Node!')
        # The interpreter of panda_widgets is used for the tuning and running of existing programs
        # The interpreter of pbd_interface is used for actions that are targeted at the program-to-be-created,
        # such as moving to previous preconditions when primitives are deleted

        self.interpreter = interpreter.PandaProgramInterpreter(robotless_debug=self.robotless_debug)  # internal interpreter, for execute_now_primitives
        self.interpreter.loaded_program = self.program
        self.freeze()

    def initialize_program(self):
        # TODO: maybe enforce some check on the rest of the functions? to check for initialization
        if not self.robotless_debug:
            while self.last_pose is None or self.last_gripper_width is None:
                rospy.sleep(1.0)

            was_relaxed = self.relaxed
            if was_relaxed:
                self.freeze()

            self.program.save_arm_state(self.last_pose)
            # TODO: assumption here, I am not grasping at the beginning...
            self.program.save_gripper_state(pp.GripperState(self.last_gripper_width, 0.0))

            if was_relaxed:
                self.relax()
        else:
            self.last_pose = np.random.uniform(0, 1, 3)
            self.last_gripper_width = np.random.uniform(0, 0.08, 1)
            # During robotless debug, we replace actual values with randomly generated ones
            # This way, we can still test the functionality of the GUI while not
            # connected to the robot.
            self.program.save_arm_state(self.last_pose)
            self.program.save_gripper_state(pp.GripperState(self.last_gripper_width, 0.0))
        self.program.initialized = True

    def gripper_state_callback(self, msg):
        last_gripper_width = msg.position[0] + msg.position[1]
        self.last_gripper_width = last_gripper_width if last_gripper_width <= 0.08 else 0.08
        self.last_gripper_velocity = msg.velocity

    def relax(self):
        if not self.robotless_debug:
            req = EnableTeachingRequest()
            req.ft_threshold_multiplier = self.default_parameters['kinesthestic_ft_threshold']
            req.teaching = 1

            try:
                res = self.kinesthetic_client(req)
            except rospy.ServiceException:
                rospy.logerr('Cannot contact Kinesthetic Teaching client!')
                self.last_pose = None
                return False

            if res.success:
                self.last_pose = res.ee_pose
                self.relaxed = True
            return True

        else:
            self.last_pose = np.random.uniform(0, 1, 3)
            self.relaxed = True

    def relax_only_arm(self):
        if not self.robotless_debug:
            req = EnableTeachingRequest()
            req.ft_threshold_multiplier = self.default_parameters['kinesthestic_ft_threshold']
            req.teaching = 2

            try:
                res = self.kinesthetic_client(req)
            except rospy.ServiceException:
                rospy.logerr('Cannot contact Kinesthetic Teaching client!')
                self.last_pose = None
                return False

            if res.success:
                self.last_pose = res.ee_pose
                self.relaxed = True
            return True
        else:
            self.last_pose = np.random.uniform(0, 1, 3)
            self.relaxed = True

    def relax_only_wrist(self):
        if not self.robotless_debug:
            req = EnableTeachingRequest()
            req.ft_threshold_multiplier = self.default_parameters['kinesthestic_ft_threshold']
            req.teaching = 3

            try:
                res = self.kinesthetic_client(req)
            except rospy.ServiceException:
                rospy.logerr('Cannot contact Kinesthetic Teaching client!')
                self.last_pose = None
                return False

            if res.success:
                self.last_pose = res.ee_pose
                self.relaxed = True
            return True
        else:
            self.last_pose = np.random.uniform(0, 1, 3)
            self.relaxed = True

    def relax_finger(self):
        temp_program = pp.PandaProgram()
        request = MoveFingersRequest()
        request.width = self.last_gripper_width

        move_fingers_primitive = pp.MoveFingers()
        move_fingers_primitive.set_parameter_container(request)

        temp_program.insert_primitive(move_fingers_primitive, [None, pp.GripperState(self.last_gripper_width, 0.0)])

        self.interpreter.load_program(temp_program)
        return self.interpreter.execute_rest_of_program(one_shot_execution=True)

    def freeze(self):
        if not self.robotless_debug:
            req = EnableTeachingRequest()
            req.teaching = 0

            try:
                res = self.kinesthetic_client(req)
            except rospy.ServiceException:
                rospy.logerr('Cannot contact Kinesthetic Teaching client!')
                self.last_pose = None
                return False

            if res.success:
                self.last_pose = res.ee_pose
                self.relaxed = False
            return True

        else:
            self.last_pose = np.random.uniform(0, 1, 3)
            self.relaxed = False

    def insert_move_to_ee(self):
        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        # create new Goal for the primitive
        goal = MoveToEEGoal()
        goal.pose = self.last_pose
        goal.position_speed = self.default_parameters['move_to_ee_default_position_speed']
        goal.rotation_speed = self.default_parameters['move_to_ee_default_rotation_speed']

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
        goal.position_speed = self.default_parameters['move_to_contact_default_position_speed']
        goal.rotation_speed = self.default_parameters['move_to_contact_default_rotation_speed']
        goal.force_threshold = self.default_parameters['move_to_contact_default_force_threshold']
        goal.torque_threshold = self.default_parameters['move_to_contact_default_torque_threshold']

        move_to_contact_primitive = pp.MoveToContact()
        move_to_contact_primitive.set_parameter_container(goal)
        self.program.insert_primitive(move_to_contact_primitive, [goal.pose, None])

        if was_relaxed:
            self.relax()

    def insert_user_sync(self):
        goal = UserSyncGoal()
        goal.force_threshold = self.default_parameters['user_sync_default_force_threshold']

        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        user_sync_primitive = pp.UserSync()
        user_sync_primitive.set_parameter_container(goal)
        self.program.insert_primitive(user_sync_primitive, [None, None])

        if was_relaxed:
            self.relax()

    def insert_apply_force_fingers(self):
        request = ApplyForceFingersRequest()
        request.force = self.default_parameters['apply_force_fingers_default_force']

        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        apply_force_fingers_primitive = pp.ApplyForceFingers()
        apply_force_fingers_primitive.set_parameter_container(request)

        if not self.robotless_debug:
            self.execute_primitive_now(apply_force_fingers_primitive)

        self.program.insert_primitive(apply_force_fingers_primitive, [None, pp.GripperState(self.last_gripper_width,
                                                                                            request.force)])

        if was_relaxed:
            self.relax()

    def insert_move_fingers(self):
        request = MoveFingersRequest()
        request.width = self.last_gripper_width

        was_relaxed = self.relaxed
        if was_relaxed:
            self.freeze()

        move_fingers_primitive = pp.MoveFingers()
        move_fingers_primitive.set_parameter_container(request)
        self.program.insert_primitive(move_fingers_primitive, [None, pp.GripperState(self.last_gripper_width, 0.0)])

        if not self.robotless_debug:
            self.execute_primitive_now(move_fingers_primitive)

        if was_relaxed:
            self.relax()

    def execute_primitive_now(self, primitive):
        temp_program = pp.PandaProgram()
        # TODO: is it worth to investigate this copy bug? with the deepcopy the code works as intended...
        temp_program.insert_primitive(copy.deepcopy(primitive), [None, None])

        self.interpreter.load_program(temp_program)
        return self.interpreter.execute_rest_of_program(one_shot_execution=True)
