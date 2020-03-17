import numpy as np
from scipy.optimize import minimize
import os
import pickle
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

class Segmentation():

    def __init__(self):
        self.trajectory_points = []
        self.points_to_segment = []
        self.time_axis = []
        self.velocities = []
        self.timeStep = 0.1
        self.normFactor = 0.20
        self.devFactor = 0.06
        self.penalizingFactor = 10000.0
        self.L_min = 0.10
        self.M_min = 10
        self.d = 0.002
        self.m = int(self.L_min/(self.d * 5))

    def costFunction(self, a_j):
        t_ij = (self.points_to_segment - a_j[3:6]).dot(a_j[0:3])
        size = t_ij.size
        t_ij = np.reshape(t_ij, (size, 1))
        prediction = np.dot(t_ij, a_j[0:3].reshape(1, 3)) + a_j[3:6]
        error = np.linalg.norm(self.points_to_segment[1:] - prediction[:-1], axis=1)
        value =  1 - np.power(np.exp((-1)*error/self.normFactor), 2)
        return sum(value) + self.penalizingFactor

    def optimize(self, a_j, fn):
        result = minimize(fn, a_j, method='nelder-mead')
        #sample = self.costFunction(np.array(result.x))
        return result.x

    def initialGuess(self, a_j):
        t_ij = (self.points_to_segment[0] - a_j[3:6]).dot(a_j[0:3])
        size = t_ij.size
        t_ij = np.reshape(t_ij, (size, 1))
        prediction = np.dot(t_ij, a_j[0:3].reshape(1, 3)) + a_j[3:6]
        if len(self.points_to_segment) > self.m:
            point_at_m = self.m
        else:
            point_at_m = len(self.points_to_segment) - 1    
        error = np.linalg.norm(self.points_to_segment[point_at_m] - prediction, axis=1)
        value =  1 - np.power(np.exp((-1)*error/self.normFactor), 2)
        return sum(value) + self.penalizingFactor

    def downSamplePoints(self):
        self.downSamplePoints = []
        self.downSampleIndexes = []
        self.downSamplePoints.append(self.trajectory_points[0])
        self.downSampleIndexes.append(0)
        count = 1
        for point in self.trajectory_points[1:]:
            dist = np.linalg.norm(self.downSamplePoints[-1] - point)
            if 0.95*self.d < dist and dist > 1.05*self.d:
                self.downSamplePoints.append(point)
                self.downSampleIndexes.append(count)
            count += 1    
        #self.trajectory_points = self.downSamplePoints 

    def calculate_aj(self):
        a_start = np.array([1, 1, 1, 1, 1, 1])
        a_j_init = self.optimize(a_start, self.initialGuess)
        a_j = self.optimize(a_j_init, self.costFunction)
        return a_j_init

    def createSegments(self):
        self.meanVelocity = np.mean(self.velocities)
        j = 1
        j_start = 1
        j_end = len(self.downSamplePoints)
        L_c = 0 #cumulative value of L
        breakpoints = []
        start, end = None, None
        done = False
        k = 0
        self.points_to_segment = self.downSamplePoints
        a_j = self.calculate_aj()
        
        while done == False:
            for i in range(j, j_end):
                prediction = np.dot((self.points_to_segment[i - 1] - a_j[3:6]).dot(a_j[0:3]), a_j[0:3]) + a_j[3:6]
                #print(len(self.points_to_segment), i-k, k, i, j_end)
                deviation = np.linalg.norm(self.points_to_segment[i] - prediction)
                relativeVelocity = self.velocities[self.downSampleIndexes[i+k]]/self.meanVelocity
                #print(deviation, i + k, relativeVelocity, self.devFactor*relativeVelocity, start)
                if deviation > (self.devFactor*relativeVelocity) and start == None:
                    start = i + k
                elif deviation < (self.devFactor*relativeVelocity):
                    start = None

                if start != None and ((i+k) - start > self.m):
                    break

                if i == j_end - 1: 
                    break       

            if start != None:
                breakpoints.append(start)
                prev_start = k
                k = start
                self.points_to_segment = self.points_to_segment[((start - prev_start) + 1):]
                #print(start, len(self.points_to_segment))
                if len(self.points_to_segment) < 2:
                    done = True
                    break

                a_j = self.calculate_aj()
                j_end = len(self.points_to_segment)
                start = None
            else:
                done = True 
        result = []
        #breakpoints = self.cleanBreakpoints(breakpoints)           
        for value in breakpoints:
            print(value, self.time_axis[self.downSampleIndexes[value]]) 
            result.append(self.downSampleIndexes[value])
        return result                         

    def cleanBreakpoints(self, breakpoints):
        finalBreakpoints = []
        finalBreakpoints.append(breakpoints[0])
        for value in breakpoints[1:]:
            point = self.trajectory_points[self.downSampleIndexes[value]]
            prev_point = self.trajectory_points[self.downSampleIndexes[finalBreakpoints[-1]]]
            dist = np.linalg.norm(point - prev_point)
            if dist > 0.05:
                finalBreakpoints.append(value)
        return finalBreakpoints        

    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program 


if __name__ == '__main__':
    seg = Segmentation()
    data = seg.loadData('~/Thesis/src/eupanda/resources/data', 'pickandplace_100hz.pkl')
    #print(data["time_axis_ee"][30:35])
    seg.time_axis = data["time_axis_ee"]
    seg.velocities = data["ee_velocities"]
    traj_points = [item[0] for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    seg.downSamplePoints() 
    points = seg.createSegments()
    x = seg.trajectory_points[:, 0]
    y = seg.trajectory_points[:, 1]
    z = seg.trajectory_points[:, 2]
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(x, y, z)
    ax.plot(x[points], y[points], z[points], markersize=8, marker='d', markerfacecolor='r', linestyle='none')
    ax.plot([x[0]], [y[0]], [z[0]], markersize=8, marker='o', markerfacecolor='green', linestyle='none')
    ax.plot([x[-1]], [y[-1]], [z[-1]], markersize=8, marker='o', markerfacecolor='yellow', linestyle='none')
    ax.set_zlim(0, 0.5)
    ax.view_init(40, -10)
    fig.show()
    fig.savefig("segmentpoints_PaP_relativeVel.png")
    while True:
       pass 