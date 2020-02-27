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
        self.timeStep = 0.25
        

    def createGraph(self):
        graphWidget = pg.PlotWidget()
        return graphWidget

    def startRecording(self, dataLine, graph):
        #graph.clear()
        self.worker = RecordingWorker(self.recordData, dataLine)
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

    def calculateDistance(self):
        a = np.array(self.pose)
        b = np.array(self.previous_pose)
        dist = np.linalg.norm(a - b)
        return dist

    def recordData(self, dataLine):
        self.recording = True
        if len(self.time_axis) == 0:
            currTime = 0
        else:
            currTime = self.time_axis[-1]    
        while self.recording:
            if not self.interface.robotless_debug:
                self.trajectory_points.append(self.pose)
                self.previous_pose = self.pose
                self.poseRequest()
                dist = self.calculateDistance()
                newVelocity = dist/self.timeStep
                gripperstate = self.last_gripper_width
                self.gripper_states.append(gripperstate)
            else:    
                newVelocity = random.uniform(0, 0.25)
            self.ee_velocities.append(newVelocity)
            self.time_axis.append(currTime)
            currTime += self.timeStep
            dataLine.setData(self.time_axis, self.ee_velocities)
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
            
            