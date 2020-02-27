import pyqtgraph as pg
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QSize, QThreadPool
import random
from time import sleep
from panda_pbd.srv import EnableTeaching, EnableTeachingRequest
import rospy
import numpy as np


class Datarecorder():

    def __init__(self, interface):
        self.ee_velocities = []
        self.gripper_velocities = []
        self.gripper_states = []
        self.trajectory_points = []
        self.time_axis = []
        self.recording = False
        self.recordingThreadpool=QThreadPool()
        self.interface = interface
        self.timeStep = 0.1
        self.pose = None
        self.gripperstate = None
        

    def startRecording(self, dataLine_v, dataLine_g):
        self.worker = RecordingWorker(self.recordData, dataLine_v, dataLine_g)
        self.recordingThreadpool.start(self.worker)

    def stopRecording(self):
        self.recording = False 

    def poseRequest(self):
        req = EnableTeachingRequest()
        req.ft_threshold_multiplier = self.interface.default_parameters['kinesthestic_ft_threshold']
        req.teaching = 1

        try:
            res = self.interface.kinesthetic_client(req)
        except rospy.ServiceException:
            rospy.logerr('Cannot contact Kinesthetic Teaching client!')
            self.pose = None

        if res.success:
            self.pose = res.ee_pose       

    def calculateGripperVelocity(self):
        v = abs(self.gripperstate - self.previous_gripper_state)/self.timeStep
        return v


    def clearPlot(self, graphs):
        for graph in graphs:
            graph.clear()
        self.ee_velocities = []
        self.gripper_velocities = []
        self.time_axis = []
        self.trajectory_points = []
        self.gripper_states = [] 
        self.previous_gripper_state = None
        self.previous_pose = None              

    def calculateDistance(self):
        a = np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z])
        b = np.array([self.previous_pose.pose.position.x, self.previous_pose.pose.position.y, self.previous_pose.pose.position.z])
        dist = np.linalg.norm(a - b)
        print(a, b, dist)
        return dist

    def recordData(self, dataLine_v, dataLine_g):
        self.interface.relax()
        self.recording = True
        if len(self.time_axis) == 0:
            currTime = 0
        else:
            currTime = self.time_axis[-1]    
        while self.recording:
            if not self.interface.robotless_debug:
                self.trajectory_points.append(self.pose)
                self.gripper_states.append(self.gripperstate)
                self.previous_pose = self.pose
                self.poseRequest()
                if self.previous_pose != None:
                    dist = self.calculateDistance()
                    newVelocity = dist/self.timeStep
                else:
                    newVelocity = 0    

                self.previous_gripper_state = self.gripperstate
                self.gripperstate = self.interface.last_gripper_width   
                if self.previous_gripper_state != None:
                    gripperVelocity = self.calculateGripperVelocity()
                else:
                    gripperVelocity = 0                    
            else:    
                newVelocity = random.uniform(0, 0.25)
            self.ee_velocities.append(newVelocity)
            self.gripper_velocities.append(gripperVelocity)
            self.time_axis.append(currTime)
            currTime += self.timeStep
            dataLine_v.setData(self.time_axis, self.ee_velocities)
            dataLine_g.setData(self.time_axis, self.gripper_velocities)
            sleep(self.timeStep)

class RecordingWorker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super(RecordingWorker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs 

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        #result = self.fn(*self.args, **self.kwargs)
        try:
            result = self.fn(*self.args, **self.kwargs)
        except Exception as e:
            print(e)              
            
            