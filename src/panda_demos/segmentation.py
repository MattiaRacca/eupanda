from panda_eup.pbd_interface import PBDInterface
from panda_eup.panda_primitive import PandaProgram
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import MoveFingersRequest, ApplyForceFingersRequest
import panda_primitive as pp
from gripper_segmentation import GripperSegmentation
from trajectory_segmentation import TrajSeg


class Segmentation():

    def __init__(self, interface):
        self.program = PandaProgram()
        self.data = None
        self.interface = interface

    def createSegments(self):
        traj_points = [item[0] for item in self.data["trajectory_points"]]
        self.trajectory_points = np.array(traj_points)
        self.gripper_states = self.data["gripper_states"]
        self.gripper_velocities = self.data["gripper_velocities"]
        self.time_axis_ee = self.data["time_axis_ee"]
        self.time_axis_gripper = self.data["time_axis_gripper"]
        
        gripSeg = Grippersegmentation()
        gripSeg.trajectory_points = self.trajectory_points
        gripSeg.gripper_states = self.gripper_states
        gripSeg.gripper_velocities = self.gripper_velocities
        gripSeg.time_axis_ee = self.time_axis_ee
        gripSeg.time_axis_gripper = self.time_axis_gripper

        trajSeg = TrajSeg()

        ma = gripSeg.moving_average(gripSeg.gripper_velocities)
        segments = gripSeg.createSegments(ma)

        prevEnd = None
        for segment in segments:
            start = segment[0]
            end = segment[1]
            if prevEnd != None:
                self.addGripperAction(prevEnd, start)
            trajSeg.trajectory_points = self.trajectory_points[start:end]
            trajSeg.initialize()
            result = trajSeg.optimize()
            result = [item+start for item in result]
            motionStart = start
            #for point in result:

    def addGripperAction(self, start, end):
        startwidth = self.gripper_states[start]
        endwidth = self.gripper_states[end]
        if endwidth < startwidth:
            self.addFingerGrasp(endwidth)
        else:
            self.addMoveFingers(endwidth)    

    def addFingerGrasp(self, width):
        request = ApplyForceFingersRequest()
        request.force = self.interface.default_parameters['apply_force_fingers_default_force']

        apply_force_fingers_primitive = pp.ApplyForceFingers()
        apply_force_fingers_primitive.set_parameter_container(request)
        self.program.insert_primitive(apply_force_fingers_primitive, [None, pp.GripperState(width, request.force)])

    def addMoveFingers(self, width):
        request = MoveFingersRequest()
        request.width = width

        move_fingers_primitive = pp.MoveFingers()
        move_fingers_primitive.set_parameter_container(request)
        self.program.insert_primitive(move_fingers_primitive, [None, pp.GripperState(width, 0.0)])

    def addLinearMotion(self, end):
        goal = MoveToEEGoal()
        goal.pose = end
        goal.position_speed = self.interface.default_parameters['move_to_ee_default_position_speed']
        goal.rotation_speed = self.interface.default_parameters['move_to_ee_default_rotation_speed']

        move_to_ee_primitive = pp.MoveToEE()
        move_to_ee_primitive.set_parameter_container(goal)
        self.interface.program.insert_primitive(move_to_ee_primitive, [goal.pose, None])


    def getPose(self, index):
        return self.data["trajectory_points"][index]

    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program                