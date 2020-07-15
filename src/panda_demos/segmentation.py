from panda_eup.pbd_interface import PandaPBDInterface
from panda_eup.panda_primitive import PandaProgram
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal
from panda_pbd.srv import MoveFingersRequest, ApplyForceFingersRequest
import panda_eup.panda_primitive as pp
from gripper_segmentation import GripperSegmentation
from trajectory_segmentation import TrajSeg
import os
import pickle
import numpy as np
import time


class Segmentation():

    def __init__(self, interface):
        self.data = None
        self.interface = interface
        #self.interface.initialize_program()
        self.max_deviation = 0.10

    def createSegments(self):
        start_time = time.time()
        traj_points = [item.pose.position for item in self.data["trajectory_points"]]
        self.trajectory_points = np.array(traj_points)
        self.gripper_states = self.data["gripper_states"]
        self.ee_velocities = self.data["ee_velocities"]
        self.interface.program.save_arm_state(self.data["trajectory_points"][0])
        initial_gripper_state = next(item for item in self.gripper_states if item is not None)
        self.interface.program.save_gripper_state(pp.GripperState(initial_gripper_state, 0.0))
        self.gripper_velocities = self.data["gripper_velocities"]
        self.time_axis_ee = self.data["time_axis_ee"]
        self.time_axis_gripper = self.data["time_axis_gripper"]
        
        gripSeg = GripperSegmentation()
        gripSeg.trajectory_points = self.trajectory_points
        gripSeg.gripper_states = self.gripper_states
        gripSeg.gripper_velocities = self.gripper_velocities
        gripSeg.time_axis_ee = self.time_axis_ee
        gripSeg.time_axis_gripper = self.time_axis_gripper

        trajSeg = TrajSeg(self.max_deviation)

        ma = gripSeg.moving_average(gripSeg.gripper_velocities)
        segments = gripSeg.createSegments(ma)

        prevEnd = None
        motionStart = 0
        for segment in segments:
            start = segment[0]
            end = segment[1]
            if prevEnd != None:
                self.addGripperAction(prevEnd, start)
            trajSeg.trajectory_points = self.trajectory_points[start:end]
            trajSeg.initialize()
            result = trajSeg.optimize()
            result = [item+start for item in result]
            prevpoint = start
            for point in result:
                vel = self.getAverageVelocity(prevpoint, point)
                endpoint = self.data["trajectory_points"][point]
                self.addLinearMotion(endpoint, vel)
                prevpoint = point     
            prevEnd = end    
        print("--- %.2f seconds ---" % (time.time() - start_time))
        print("--- %s primitives ---" % len(self.interface.program.primitives))

    def getAverageVelocity(self, start, end):
        velocities = self.ee_velocities[start:end]
        vel = round(np.mean(velocities), 2)
        vel = min(vel, 0.35) #cap velocity at 0.35 m/s
        return vel

    def addGripperAction(self, start, end):
        time_start = self.time_axis_ee[start]
        diff = abs(np.array(self.time_axis_gripper) - time_start)
        startidx = diff.argmin()

        time_end = self.time_axis_ee[end]
        diff = abs(np.array(self.time_axis_gripper) - time_end)
        endidx = diff.argmin()

        startwidth = self.gripper_states[startidx]
        endwidth = self.gripper_states[endidx]
        if endwidth < startwidth:
            self.addFingerGrasp(endwidth)
        else:
            self.addMoveFingers(endwidth)    

    def addFingerGrasp(self, width):
        request = ApplyForceFingersRequest()
        request.force = self.interface.default_parameters['apply_force_fingers_default_force']

        apply_force_fingers_primitive = pp.ApplyForceFingers()
        apply_force_fingers_primitive.set_parameter_container(request)
        self.interface.program.insert_primitive(apply_force_fingers_primitive, [None, pp.GripperState(width, request.force)])

    def addMoveFingers(self, width):
        request = MoveFingersRequest()
        request.width = width

        move_fingers_primitive = pp.MoveFingers()
        move_fingers_primitive.set_parameter_container(request)
        self.interface.program.insert_primitive(move_fingers_primitive, [None, pp.GripperState(width, 0.0)])

    def addLinearMotion(self, end, vel):
        goal = MoveToEEGoal()
        goal.pose = end
        #goal.position_speed = self.interface.default_parameters['move_to_ee_default_position_speed']
        goal.position_speed = vel
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

    def saveProgram(self, path, filename):
        self.interface.program.dump_to_file(filepath=path, filename=filename)

if __name__ == '__main__':
    interface = PandaPBDInterface(robotless_debug = True)
    seg = Segmentation(interface)
    seg.data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PaP_1.pkl")
    seg.createSegments()
    seg.saveProgram(path=~'/Thesis/src/eupanda/resources', filename="segmentation_test.pkl")
