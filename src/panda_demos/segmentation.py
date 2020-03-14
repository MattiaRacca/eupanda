import numpy as np
from scipy.optimize import minimize
import os
import pickle

class Segmentation():

    def __init__(self):
        self.trajectory_points = []
        self.points_to_segment = []
        self.time_axis = []
        self.timeStep = 0.1
        self.normFactor = 0.20
        self.devFactor = 0.10
        self.penalizingFactor = 10000.0
        self.L_min = 0.15
        self.M_min = 10
        self.d = 0.002

    def costFunction(self, a_j):
        t_ij = (self.points_to_segment - a_j[3:6]).dot(a_j[0:3])
        size = t_ij.size
        t_ij = np.reshape(t_ij, (size, 1))
        prediction = np.dot(t_ij, a_j[0:3].reshape(1, 3)) + a_j[3:6]
        error = np.linalg.norm(self.points_to_segment[1:] - prediction[:-1], axis=1)
        value =  1 - np.power(np.exp((-1)*error/self.normFactor), 2)
        return sum(value) + self.penalizingFactor

    def optimize(self, a_j):
        result = minimize(self.costFunction, a_j, method='nelder-mead')
        sample = self.costFunction(np.array(result.x))
        return result.x

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
        self.trajectory_points = self.downSamplePoints 

    def createSegments(self):
        j = 1
        j_start = 1
        j_end = len(self.trajectory_points)
        L_c = 0 #cumulative value of L
        breakpoints = []
        start, end = None, None
        done = False
        self.points_to_segment = self.trajectory_points

        a_j_init = np.array([1, 1, 1, 1, 1, 1])
        a_j = self.optimize(a_j_init)
        
        while done == False:
            for i in range(j, j_end):
                prediction = np.dot((self.trajectory_points[i - 1] - a_j[3:6]).dot(a_j[0:3]), a_j[0:3]) + a_j[3:6]
                deviation = np.linalg.norm(self.trajectory_points[i] - prediction)
                print(deviation, i, self.time_axis[self.downSampleIndexes[i]])
                if deviation > self.devFactor and start == None:
                    start = i
                elif deviation < self.devFactor and start != None:
                    end = i
                    if (end - start) > (self.L_min/(self.d*5)):
                        break
                    else:
                        start = None
                if i == j_end - 1:
                    end = None        

            if start != None:
                breakpoints.append(start)
                self.points_to_segment = self.points_to_segment[(start + 1):]
                if len(self.points_to_segment) < 2:
                    done = True
                    break
                a_j = self.optimize(a_j_init)
                j = start + 1
                start = None
                end = None
            else:
                done = True    
        for value in breakpoints:
            print(value, self.time_axis[self.downSampleIndexes[value]])                  
        '''                
                L_c += deviation
                if i == j_end - 1:
                    done = True
                if L_c > self.L_min or deviation > self.M_min:
                    break        
            breakpoints.append(i)
            L_c = 0
            self.points_to_segment = self.points_to_segment[i:]
            if len(self.points_to_segment) > 1:
                a_j = self.optimize(a_j)
            else:
                done = True    
            j = i
           
        for value in breakpoints:
            print(value, self.time_axis[value]) 
        '''         

        
    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program 


if __name__ == '__main__':
    seg = Segmentation()
    data = seg.loadData('~/Thesis/src/eupanda/resources/data', '4_motions_100hz.pkl')
    #print(data["time_axis_ee"][30:35])
    seg.time_axis = data["time_axis_ee"]
    traj_points = [item[0] for item in data["trajectory_points"]]
    seg.trajectory_points = np.array(traj_points)
    seg.downSamplePoints()
    #seg.optimize(np.array([1, 1, 1, 1, 1, 1])) 
    seg.createSegments() 