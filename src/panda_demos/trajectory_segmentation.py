import numpy as np
import os
import pickle
from copy import deepcopy

class TrajSeg():
    
    def __init__(self, max_deviation):
        self.trajectory_points = []
        self.d = 0.02 #Distance threshold for downsampling
        self.max_deviation = max_deviation #Maximum allowed deviation
        
    def initialize(self):
        self.downSample()
        self.points_to_segment = self.downSamplePoints
    
    def downSample(self):
        '''
        Perform a distance-wise downsampling to the set of trajectory points using parameter self.d as the distance threshold
        '''
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

        print("--- Number of Downsampled trajectory points: %s ---" % len(self.downSamplePoints))      
    
    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program
    
    def optimize(self):
        finalCosts = []
        prevSolution = []
        N = len(self.points_to_segment)
        #Return empty array if the segment is too short
        if N < 3:
            print("No segments returned due to the length of the given trajectory")
            return []

        initialCosts = []
        for point in range(1, N):
            #Calculate summed and maximum deviation for the segment from the first point to points in range (1,N)
            d, maxd = self.calculateTransitionCost(0, point)
            initialCosts.append((d, [point], maxd))
        
        #End execution if one segment was enough to fulfill the condition of maximum allowed deviation
        if maxd < self.max_deviation:
            result = []
            for point in initialCosts[-1][1]:
                result.append(self.downSampleIndexes[point])    
            print("--- Maximum absolute deviation: %s ---" % maxd)    
            return result

        storedCosts = []
        prevCosts = initialCosts
        #i represents the number of segments
        for i in range(2, len(self.points_to_segment)):
            costs = []
            #j represents index of a trajectory point                    
            for j in range(i, len(self.points_to_segment)):
                values = []
                #Each route from the first trajectory point to point j is considered. E.g. if j = 5, it can be reached with route 1-3-5 or 1-4-5
                #when using 2 segments
                for point in range(i-1, j):
                    d, maxd = self.calculateTransitionCost(point, j)
                    cost = d + prevCosts[point - (i-1)][0] #Add the summed deviation of the last segment to the previous sum for the rest of the segments
                    indexes = deepcopy(prevCosts[point - (i-1)][1]) #Keep track of trajectory point indexes which were used to obtain the cost above
                    if maxd <= prevCosts[point - (i-1)][2]: #Update maximum deviation of the current segments
                        maxd = prevCosts[point - (i-1)][2]
                    indexes.append(j)
                    values.append((cost, indexes, maxd))
                minValue = min(values) #Pick only optimal route to reach j, as in route with minimal summed deviation
                costs.append(minValue)
            storedCosts.append(costs)
            finalCosts.append(costs[-1])
            prevCosts = costs
            #Check the last option of costs, covering trajectory from the first point to the last point, whether it fulfills
            #the maximum absolute deviation condition
            if costs[-1][2] < self.max_deviation:
                print("--- Maximum absolute deviation: %s ---" % round(costs[-1][2], 3)) 
                break
            else:
                prevSolution = costs[-1][1]
        result = []
        #Return solution with respect to trajectory points of the full, non-downsampled trajectory.
        for point in costs[-1][1]:
            result.append(self.downSampleIndexes[point])
        return result         

    def calculateTransitionCost(self, startIndex, endIndex):
        '''
        Calculate maximum and summed deviation for the segment from startIndex to endIndex
        '''
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
    seg = TrajSeg(0.10)
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "PoseStamped_4motions.pkl")
    seg.time_axis = data["time_axis_ee"]
    seg.velocities = data["ee_velocities"]
    traj_points = [item.pose.position for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    #seg.trajectory_points = seg.trajectory_points[966:1680]
    seg.initialize()
    res = seg.optimize()
    print(res)