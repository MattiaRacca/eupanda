import numpy as np
import os
import pickle
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from trajectory_segmentation import TrajSeg

class Grippersegmentation():
    
    def __init__(self):
        self.trajectory_points = []
        self.gripper_states = []
        self.gripper_velocities = []
    
    def moving_average(self, values, n=3):
        return np.convolve(values, np.ones(n), 'valid') / n
    
    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program
    
    def findGripperActions(self, data):
        threshold = 0.01
        fingerGrasps = data < -1*threshold
        moveFingers = data > threshold
        fingerGrasp_edges = np.convolve([1, -1], fingerGrasps, mode='same')
        moveFingers_edges = np.convolve([1, -1], moveFingers, mode='same')
        fingerGrasp_indices = [item for item in zip(np.where(fingerGrasp_edges==1)[0], np.where(fingerGrasp_edges==-1)[0])]
        moveFingers_indices = [item for item in zip(np.where(moveFingers_edges==1)[0], np.where(moveFingers_edges==-1)[0])]
        
        for item in fingerGrasp_indices:
            timestamp_start = self.time_axis_gripper[item[0]]
            timestamp_end = self.time_axis_gripper[item[1]]
            diff_start = abs(np.array(self.time_axis_ee) - timestamp_start)
            diff_end = abs(np.array(self.time_axis_ee) - timestamp_end)
            idx = diff_start.argmin()
        
        return fingerGrasp_indices, moveFingers_indices

    def createSegments(self, data):
        done = False
        start = 0
        fingerGrasp_indices, moveFingers_indices = self.findGripperActions(data)
        trajSeg = TrajSeg()
        while not done:
            if len(fingerGrasp_indices) == 0 and len(moveFingers_indices) == 0:
                done = True
            elif len(fingerGrasp_indices) == 0:
                currentAction = moveFingers_indices.pop(0)
            elif len(moveFingers_indices) == 0:
                currentAction = fingerGrasp_indices.pop(0)
            elif fingerGrasp_indices[0][0] < moveFingers_indices[0][0]:
                currentAction = fingerGrasp_indices.pop(0)
            else:
                currentAction = moveFingers_indices.pop(0)

            if not done:    
                gripper_timestamp = self.time_axis_gripper[currentAction[0]]
                diff = abs(np.array(self.time_axis_ee) - gripper_timestamp)
                idx = diff.argmin()
                points_to_segment = self.trajectory_points[start:idx]
            else:
                idx = len(self.trajectory_points) - 1      
                points_to_segment = self.trajectory_points[start:]
            trajSeg.trajectory_points = np.array(points_to_segment)
            trajSeg.initialize()
            result = trajSeg.optimize()
            points = []
            points.append(start)
            for point in result[1:-1]:
                value = start + trajSeg.downSampleIndexes[point]
                points.append(value)
            points.append(idx)    
            #run trajectory_segmentation with points_to_segment
            print(start, idx)
            print(result, points)
            if not done:
                end_timestamp = self.time_axis_gripper[currentAction[1]]
                diff = abs(np.array(self.time_axis_ee) - end_timestamp)
                start = diff.argmin()

if __name__ == '__main__':
    seg = Grippersegmentation()
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PaP_1.pkl")
    traj_points = [item[0] for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    seg.gripper_states = data["gripper_states"]
    seg.gripper_velocities = data["gripper_velocities"]
    seg.time_axis_gripper = data["time_axis_gripper"]
    seg.time_axis_ee = data["time_axis_ee"]
    ma = seg.moving_average(seg.gripper_velocities)
    #seg.findGripperActions(ma)
    seg.createSegments(ma)
    #ma = seg.gripper_velocities
    count = 0
    for value in ma:
        #print(value, seg.gripper_states[count + 1])
        count += 1