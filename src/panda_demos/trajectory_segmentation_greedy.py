import numpy as np
import os
import pickle

class TrajSeg_greedy():
    
    def __init__(self, max_deviation):
        self.trajectory_points = []
        self.d = 0.02 #distance threshold for downsampling
        self.max_deviation = max_deviation #Maximum allowed deviation
        
    def initialize(self):
        self.downSample()
        self.points_to_segment = self.downSamplePoints

    def optimize(self):
        #Return empty array if the segment is too short
        if len(self.points_to_segment) < 3:
            return []
        self.points = np.arange(len(self.points_to_segment))
        self.remainingPoints = np.ones(len(self.points_to_segment))
        costs = []
        #As the points-to-be-removed are chosen based on minimum transition cost, we assign large interger values for the costs of points
        #which should not be removed or which already have been removed to prevent accidently picking them
        costs.append(10000)
        done = False
        prevDeleted = None
        #Calculate transition cost for each point i
        for i in range(1, len(self.points) - 1):
            cost = self.calculateTransitionCost(i)
            costs.append(cost)
        costs.append(10000)    
            
        while not done:
            breakpoints = [item for i,item in enumerate(self.points) if self.remainingPoints[i] == True] #List of current segmentation points
            distances = self.calculatePerformance(breakpoints) #Calculate deviation for each trajectory point from the proposed trajectory
            totalCost = sum(distances)
            maxDev = max(distances)
            numOfPoints = sum(self.remainingPoints) - 2
            
            #Compared the current maximum deviation with maximum allowed deviation, and end the algorithm if maximum allowed deviation is exceeded.
            if maxDev > self.max_deviation:
                done = True
                self.remainingPoints[prevDeleted] = True
                print("--- Maximum absolute deviation: %s ---" % round(prevMaxDev, 3)) 
                break
            
            minCostIndex = np.argmin([item for item in costs]) #find index for item with minimum transition cost
            costs[minCostIndex] = 10**4
            self.remainingPoints[minCostIndex] = False
            #Save the index of previously deleted segmentation point and the maximum absolute deviation with that segmentation point
            prevDeleted = minCostIndex
            prevMaxDev = maxDev
            
            #Stop execution if all segmentation points have been removed
            if all(x==costs[0] for x in costs):
                done = True
                breakpoints = [0, len(self.points_to_segment) - 1]
                lastmaxdev = max(self.calculatePerformance(breakpoints))
                if lastmaxdev > self.max_deviation:
                    self.remainingPoints[prevDeleted] = True
                    print("--- Maximum absolute deviation: %s ---" % round(prevMaxDev, 3))    
                else:
                    print("--- Maximum absolute deviation: %s ---" % round(lastmaxdev, 3))
                break

            #Find previus and following segmentation point from the one that was just removed
            diff = self.points - minCostIndex
            a = np.max([item for i, item in enumerate(diff) if item < 0 and self.remainingPoints[i] == True])
            prevIndex = np.where(diff == a)[0].item()
            nearestPrevious = self.points[prevIndex]
            a = np.min([item for i, item in enumerate(diff) if item > 0 and self.remainingPoints[i] == True])
            folIndex = np.where(diff == a)[0].item()
            nearestFollowing = self.points[folIndex]
            
            #Calculate transition cost again for previous and following segmentation point
            if nearestPrevious > 0:
                costPrevious = self.calculateTransitionCost(nearestPrevious)
                costs[nearestPrevious] = costPrevious
            if nearestFollowing < len(self.points_to_segment) - 1:    
                costFollowing = self.calculateTransitionCost(nearestFollowing)
                costs[nearestFollowing] = costFollowing

        #Present the indexes of finalPoints with respect to full, non-downsampled trajectory
        self.remainingPoints = self.remainingPoints[1:]
        result = [item for i, item in enumerate(self.points[1:]) if self.remainingPoints[i] == True]
        finalPoints = []
        for point in result:
            finalPoints.append(self.downSampleIndexes[point])
        return finalPoints       

    def calculateTransitionCost(self, i):
        '''
        Calculate transition cost for segmentation point i meaning the summed deviation of trajectory points from the previous segmentation point of i
        to the following segmentation point of i.
        '''
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
            #Set limit at 0.85*self.d to allow some error
            if dist > 0.85*self.d:
                self.downSamplePoints.append(point)
                self.downSampleIndexes.append(count)
                distances.append(dist)
            count += 1
        
        print("--- Number of Downsampled trajectory points: %s ---" % len(self.downSamplePoints))        
    
    def calculatePerformance(self, breakpoints):
        '''
        Calculate deviation for each trajectory point from the proposed trajectory that is formed using the segmenation points of array breakpoints
        '''
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
    seg = TrajSeg_greedy(0.10)
    data = seg.loadData("~/Thesis/src/eupanda/resources/data", "MaxDev_3motions.pkl")
    seg.time_axis = data["time_axis_ee"]
    seg.velocities = data["ee_velocities"]
    traj_points = [item.pose.position for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    #seg.trajectory_points = seg.trajectory_points[966:1680]
    seg.initialize()
    result = seg.optimize()
    print(result)