import numpy as np
from scipy.optimize import minimize

class Segmentation():

    def __init__(self):
        self.trajectory_points = []
        self.timeStep = 0.1
        self.normFactor = 3
        self.penalizingFactor = 10000

    def costFunction(self, a_j):
        t_ij = a_j[0:3].dot(traj_points - a_j[3:6])
        prediction = a_j[0:3]*t_ij + a_j[3:6]
        error = np.linalg.norm(traj_points - prediction)
        value =  1 - np.exp(np.power((-1)*error/self.normFactor, 2))
        return sum(sum(value) + self.penalizingFactor)

    def createSegments(self):
        j = 1
        j_start = 1
        j_end = len(self.trajectory_points)
        L_min = 0.10
        M_min = 0.05

        self.traj_points = np.transpose(np.array(self.trajectory_points))
        #p^(t_ij, a_j) = a_1j*t_ij + a_0j
        #t_ij = a_1j*(p_i - a_0j)
        # => p^(t_ij, a_j) = a_1j^2*p_i - a_1j^2*a_0j + a_0j
        # => nextPoint = a_1j^2*(currentPoint - a_0j) + a_0j
        # 5 = x^2(3 - y) + y
        a_j_init = np.array([1, 1, 1, 1, 1, 1])