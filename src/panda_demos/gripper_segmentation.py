import numpy as np
import os
import pickle
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from trajectory_segmentation import TrajSeg


class GripperSegmentation():

    def __init__(self):
        self.trajectory_points = []
        self.gripper_states = []
        self.gripper_velocities = []

    def moving_average(self, values, n=3):
        '''
        Perform moving average filtering with filter size n on array values
        '''
        return np.convolve(values, np.ones(n), 'valid') / n

    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program

    def findGripperActions(self, data):
        threshold = 0.01  # velocity threshold for detecting gripper actions

        #Create bool array for both Move Fingers and Finger Grasp to find values which exceed threshold or remain below -1*threshold
        fingerGrasps = data < -1 * threshold
        moveFingers = data > threshold

        #Find the start and end of each segment with convolution
        fingerGrasp_edges = np.convolve([1, -1], fingerGrasps, mode='same')
        moveFingers_edges = np.convolve([1, -1], moveFingers, mode='same')
        fingerGrasp_indices = [item for item in zip(np.where(fingerGrasp_edges == 1)[0], np.where(fingerGrasp_edges == -1)[0])]
        moveFingers_indices = [item for item in zip(np.where(moveFingers_edges == 1)[0], np.where(moveFingers_edges == -1)[0])]

        return fingerGrasp_indices, moveFingers_indices

    def createSegments(self, data):
        done = False
        start = 0
        fingerGrasp_indices, moveFingers_indices = self.findGripperActions(data)
        segments = []
        while not done:
            #Choose the next gripper action based on the indices
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
                #Find the respective trajectory point index for the current gripper action
                gripper_timestamp = self.time_axis_gripper[currentAction[0]]
                diff = abs(np.array(self.time_axis_ee) - gripper_timestamp)
                idx = diff.argmin()
                points_to_segment = self.trajectory_points[start:idx]
            else:
                idx = len(self.trajectory_points) - 1
                points_to_segment = self.trajectory_points[start:]
            segments.append((start, idx))
            if not done:
                #Assign the start of next segment to the end of previous one
                end_timestamp = self.time_axis_gripper[currentAction[1]]
                diff = abs(np.array(self.time_axis_ee) - end_timestamp)
                start = diff.argmin()
        return segments

if __name__ == '__main__':
    seg = GripperSegmentation()
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PaP_1.pkl")
    traj_points = [item[0] for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    seg.gripper_states = data["gripper_states"]
    seg.gripper_velocities = data["gripper_velocities"]
    seg.time_axis_gripper = data["time_axis_gripper"]
    seg.time_axis_ee = data["time_axis_ee"]
    ma = seg.moving_average(seg.gripper_velocities)
    segments = seg.createSegments(ma)
    print(segments)
    count = 0
    for value in ma:
        count += 1
