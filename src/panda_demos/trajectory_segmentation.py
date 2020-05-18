import numpy as np
import os
import pickle
from copy import deepcopy

class TrajSeg():
    
    def __init__(self):
        self.trajectory_points = []
        self.segmentation_points = []
        self.d = 0.02
        self.zeta = 0.3
        self.cleanup = False
        
    def initialize(self):
        self.downSamplePoints()
        self.points_to_segment = self.downSamplePoints
        self.segmentation_points.append(0)
        self.segmentation_points.append(len(self.points_to_segment) - 1)
    
    def downSamplePoints(self):
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
        initialCosts = []
        for point in range(1, N):
            d = self.calculateTransitionCost(0, point)
            initialCosts.append((d, [point]))
        
        
        storedCosts = []
        prevCosts = initialCosts
        for i in range(2, len(self.points_to_segment)):
            costs = []                    
            for j in range(i, len(self.points_to_segment)):
                values = []
                for point in range(i-1, j):
                    d = self.calculateTransitionCost(point, j)
                    #print(point)
                    cost = d + prevCosts[point - (i-1)][0]
                    indexes = deepcopy(prevCosts[point - (i-1)][1])
                    indexes.append(j)
                    values.append((cost, indexes))
                minValue = min(values) 
                costs.append(minValue)
            storedCosts.append(costs)
            finalCosts.append(costs[-1])
            prevCosts = costs
            print(len(prevCosts))
            combinedCost = costs[-1][0] + self.zeta*len(costs[-1][1])
            print(combinedCost)
            if combinedCost > prevCombinedCost:
                print(prevCombinedCost, prevSolution)
                break
            else:
                prevCombinedCost = combinedCost
                prevSolution = costs[-1][1]
        result = []
        for point in prevSolution:
            result.append(self.downSampleIndexes[point])
        return result    

    def calculateTransitionCost(self, startIndex, endIndex):
        distances = []
        start = self.points_to_segment[startIndex]
        end = self.points_to_segment[endIndex]
        for traj_point in self.points_to_segment[startIndex + 1:endIndex]:
            d = np.linalg.norm(np.cross(start-end, start-traj_point))/np.linalg.norm(end-start)
            distances.append(d)
        return sum(distances)

if __name__ == '__main__':
    seg = TrajSeg()
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PaP_1.pkl")
    seg.time_axis = data["time_axis_ee"]
    seg.velocities = data["ee_velocities"]
    traj_points = [item[0] for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    seg.trajectory_points = seg.trajectory_points[966:1680]
    seg.initialize()
    seg.optimize()