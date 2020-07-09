import numpy as np
import os
import pickle
from copy import deepcopy

class TrajSeg():
    
    def __init__(self, max_deviation):
        self.trajectory_points = []
        self.segmentation_points = []
        self.d = 0.02
        self.zeta = 0.3
        self.max_deviation = max_deviation
        self.cleanup = False
        
    def initialize(self):
        self.downSample()
        self.points_to_segment = self.downSamplePoints
        self.segmentation_points.append(0)
        self.segmentation_points.append(len(self.points_to_segment) - 1)
    
    def downSample(self):
        self.trajectory_points = [np.array([point.x, point.y, point.z]) for point in self.trajectory_points]
        self.downSamplePoints = []
        self.downSampleIndexes = []
        self.downSamplePoints.append(self.trajectory_points[0])
        self.downSampleIndexes.append(0)
        distances = []
        count = 1
        for point in self.trajectory_points[1:]:
            dist = np.linalg.norm(self.downSamplePoints[-1] - point)
            if dist > 0.85*self.d:
                self.downSamplePoints.append(point)
                self.downSampleIndexes.append(count)
                distances.append(dist)
            count += 1 
    
    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program
    
    def optimize(self):
        finalCosts = []
        prevCombinedCost = 10000
        prevSolution = []
        N = len(self.points_to_segment)
        if N < 3:
            return []
        initialCosts = []
        for point in range(1, N):
            d, maxd = self.calculateTransitionCost(0, point)
            initialCosts.append((d, [point], maxd))
        
        if maxd < self.max_deviation:
            result = []
            for point in initialCosts[-1][1]:
                result.append(self.downSampleIndexes[point])
            return result 
        storedCosts = []
        prevCosts = initialCosts
        for i in range(2, len(self.points_to_segment)):
            costs = []                    
            for j in range(i, len(self.points_to_segment)):
                values = []
                for point in range(i-1, j):
                    d, maxd = self.calculateTransitionCost(point, j)
                    cost = d + prevCosts[point - (i-1)][0]
                    indexes = deepcopy(prevCosts[point - (i-1)][1])
                    if maxd <= prevCosts[point - (i-1)][2]:
                        maxd = prevCosts[point - (i-1)][2]
                    indexes.append(j)
                    values.append((cost, indexes, maxd))
                minValue = min(values) 
                costs.append(minValue)
            storedCosts.append(costs)
            finalCosts.append(costs[-1])
            prevCosts = costs
            if costs[-1][2] < self.max_deviation:
                break
            else:
                prevSolution = costs[-1][1]
        result = []
        for point in costs[-1][1]:
            result.append(self.downSampleIndexes[point])
        return result         

    def calculateTransitionCost(self, startIndex, endIndex):
        distances = []
        start = self.points_to_segment[startIndex]
        end = self.points_to_segment[endIndex]
        for traj_point in self.points_to_segment[startIndex + 1:endIndex]:
            d = np.linalg.norm(np.cross(start-end, start-traj_point))/np.linalg.norm(end-start)
            distances.append(d)
        if len(distances) == 0:
            return 0, 0
        else:
            return sum(distances), max(distances)

if __name__ == '__main__':
    seg = TrajSeg()
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PoseStamped_4motions.pkl")
    seg.time_axis = data["time_axis_ee"]
    seg.velocities = data["ee_velocities"]
    traj_points = [item.pose.position for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    #seg.trajectory_points = seg.trajectory_points[966:1680]
    seg.initialize()
    res = seg.optimize()
    print(res)