import numpy as np
import os
import pickle

class TrajSeg():
    
    def __init__(self):
        self.trajectory_points = []
        self.segmentation_points = []
        self.d = 0.02
        self.zeta = 0.15
        
    def initialize(self):
        self.downSample()
        self.points_to_segment = self.downSamplePoints
        self.segmentation_points.append(0)
        self.segmentation_points.append(len(self.points_to_segment) - 1)

    def optimize(self):
        self.points = np.arange(len(self.points_to_segment))
        self.remainingPoints = np.ones(len(self.points_to_segment))
        costs = []
        costs.append(10000)
        done = False
        prevValue = 0
        for i in range(1, len(self.points) - 1):
            cost = self.calculateTransitionCost(i)
            costs.append(cost)
        costs.append(10000)    
            
        while not done:
            breakpoints = [item for i,item in enumerate(self.points) if self.remainingPoints[i] == True]
            totalCost = sum(self.calculatePerformance(breakpoints))
            numOfPoints = sum(self.remainingPoints) - 2
            value = totalCost
            if value > prevValue + self.zeta*numOfPoints:
                done = True
                break
            else:
                prevValue = value
            minCostIndex = np.argmin([item for item in costs])
            costs[minCostIndex] = 10**4
            self.remainingPoints[minCostIndex] = False
            diff = self.points - minCostIndex
            a = np.max([item for i, item in enumerate(diff) if item < 0 and self.remainingPoints[i] == True])
            prevIndex = np.where(diff == a)[0].item()
            nearestPrevious = self.points[prevIndex]
            a = np.min([item for i, item in enumerate(diff) if item > 0 and self.remainingPoints[i] == True])
            folIndex = np.where(diff == a)[0].item()
            nearestFollowing = self.points[folIndex]
            if nearestPrevious > 0:
                costPrevious = self.calculateTransitionCost(nearestPrevious)
                costs[nearestPrevious] = costPrevious
            if nearestFollowing < len(self.points_to_segment) - 1:    
                costFollowing = self.calculateTransitionCost(nearestFollowing)
                costs[nearestFollowing] = costFollowing

        result = [item for i, item in enumerate(self.points) if self.remainingPoints[i] == True]
        return result        
        #print(result)

    def calculateTransitionCost(self, i):
        distances = []
        diff = self.points - i
        end = np.min([item for i, item in enumerate(diff) if item > 0 and self.remainingPoints[i] == True])
        endIndex = np.where(diff == end)[0].item()
        start = np.max([item for i, item in enumerate(diff) if item < 0 and self.remainingPoints[i] == True])
        startIndex = np.where(diff == start)[0].item()
        startpoint = self.points_to_segment[startIndex]
        endpoint = self.points_to_segment[endIndex]
        for traj_point in self.points_to_segment[startIndex + 1:endIndex]:
            d = np.linalg.norm(np.cross(startpoint-endpoint, startpoint-traj_point))/np.linalg.norm(endpoint-startpoint)
            distances.append(d)
        return sum(distances)/len(distances)    

    def downSample(self):
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
    
    def calculatePerformance(self, breakpoints):
        start = 0
        distances = []
        for point in breakpoints[1:]:
            startpoint = self.points_to_segment[start]
            endpoint = self.points_to_segment[point]
            for traj_point in self.points_to_segment[start:point]:
                d = np.linalg.norm(np.cross(startpoint-endpoint, startpoint-traj_point))/np.linalg.norm(endpoint-startpoint)
                distances.append(d)
            start = point
        return distances

    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program       


if __name__ == "__main__":
    seg = TrajSeg()
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PaP_1.pkl")
    seg.time_axis = data["time_axis_ee"]
    seg.velocities = data["ee_velocities"]
    traj_points = [item[0] for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    #seg.trajectory_points = seg.trajectory_points[966:1680]
    seg.initialize()
    result = seg.optimize()
    print(result)