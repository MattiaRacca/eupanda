import pyqtgraph as pg
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QSize, QThreadPool
import random
from time import sleep
from panda_pbd.srv import EnableTeaching, EnableTeachingRequest
import rospy
import tf
import numpy as np
import pickle
import os
#from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import PoseStamped

class Datarecorder():

    def __init__(self, interface):
        self.ee_velocities = []
        self.gripper_velocities = []
        self.gripper_states = []
        self.trajectory_points = []
        self.time_axis_ee = []
        self.time_axis_gripper = []
        self.recording = False
        self.recordingThreadpool = QThreadPool()
        self.interface = interface
        self.rate = 100
        self.timeStep = rospy.Rate(self.rate)
        self.pose = None
        self.gripperstate = None
        self.data = {}
        self.listener = tf.TransformListener()
        '''
        if self.interface.robotless_debug == False:
            self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState,
                                                            self.jointStateCallback)
        '''

    def startRecording(self, dataLine_v, dataLine_g):
        self.worker = RecordingWorker(self.recordData, dataLine_v, dataLine_g)
        self.recordingThreadpool.start(self.worker)

    def stopRecording(self):
        self.recording = False 

    def saveData(self, path, filename):
        self.data["ee_velocities"] = self.ee_velocities
        self.data["gripper_velocities"] = self.gripper_velocities
        self.data["trajectory_points"] = self.trajectory_points
        self.data["gripper_states"] = self.gripper_states
        self.data["time_axis_ee"] = self.time_axis_ee
        self.data["time_axis_gripper"] = self.time_axis_gripper
        with open(os.path.join(os.path.expanduser(path), filename), 'wb') as f:
            pickle.dump(self.data, f)

    def loadData(self, path, filename):
        with open(os.path.join(os.path.expanduser(path), filename), 'rb') as f:
            loaded_program = pickle.load(f)
            return loaded_program        

    def jointStateCallback(self, msg):
        #Get URDF from franka_description
        #urdf_str = 
        pos, vel = msg.position, msg.velocity
        robot_urdf = URDF.from_xml_string(urdf_str)
        kdl_kin = KDLKinematics(robot_urdf, "panda_link0", "panda_link8")
        self.pose = kdl_kin.forward(pos)

    def getListenerValues(self):
        goal = PoseStamped()
        try:
            trans, rot = self.listener.lookupTransform("panda_link0", "panda_K", rospy.Time(0)) #Use panda_K or panda_EE for the target frame
            traj_time = rospy.Time.now()
            linear, angular = self.listener.lookupTwist("panda_link0", "panda_K", rospy.Time(0), rospy.Duration(1.0/self.rate))
            vel_time = rospy.Time.now()
            goal.header.frame_id = "panda_link0"
            goal.pose.position.x = trans[0]
            goal.pose.position.y = trans[1]
            goal.pose.position.z = trans[2]

            goal.pose.orientation.x = rot[0]
            goal.pose.orientation.y = rot[1]
            goal.pose.orientation.z = rot[2]
            goal.pose.orientation.w = rot[3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Could not get tf listener values")    
        self.trajectory_points.append(goal)
        #self.pose = goal.pose
        vel = np.sqrt(np.square(linear[0]) + np.square(linear[1]) + np.square(linear[2]))
        self.ee_velocities.append(vel)
        self.time_axis_ee.append(vel_time.to_sec() - self.start_time.to_sec())    

    '''
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
    '''
        

    def clearPlot(self, graphs):
        for graph in graphs:
            graph.clear()
        self.ee_velocities = []
        self.gripper_velocities = []
        self.time_axis_ee = []
        self.time_axis_gripper = []
        self.trajectory_points = []
        self.gripper_states = [] 
        self.previous_gripper_state = None
        self.previous_pose = None              

    '''
    def calculateDistance(self):
        a = np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z])
        b = np.array([self.previous_pose.pose.position.x, self.previous_pose.pose.position.y, self.previous_pose.pose.position.z])
        dist = np.linalg.norm(a - b)
        return dist
    '''

    def getGripperValues(self):
        self.previous_gripper_state = self.gripperstate
        self.gripperstate = self.interface.last_gripper_width
        if len(self.time_axis_gripper) > 1:
            v = (self.gripperstate - self.previous_gripper_state)/abs(self.time_axis_gripper[-1] - self.time_axis_gripper[-2])
        else:
            v = 0
        self.gripper_velocities.append(v)        
        gripper_time = rospy.Time.now()
        self.time_axis_gripper.append(gripper_time.to_sec() - self.start_time.to_sec())

    def recordData(self, dataLine_v, dataLine_g):
        count = 1
        self.interface.relax()
        self.recording = True
        if len(self.time_axis_ee) == 0:
            currTime = 0.0
        else:
            currTime = self.time_axis_ee[-1]
        self.start_time = rospy.Time.now()        
        while self.recording:
            #print(self.interface.robotless_debug)
            if not self.interface.robotless_debug:
                self.getListenerValues()
                #Gripper publish rate appears to be ~5hz, to avoid fecthing gripper values too frequently, we add
                #variable count to reduce the rate at which gripper values are added
                if count == int(self.rate/4.5):
                    self.gripper_states.append(self.gripperstate)
                    self.getGripperValues()
                    count = 1
                count += 1                       
            else:    
                newVelocity = random.uniform(0, 0.25)
                gripperVelocity = random.uniform(0, 0.05)
                self.ee_velocities.append(newVelocity)
                self.gripper_velocities.append(gripperVelocity)
                self.time_axis_ee.append(currTime)
                self.time_axis_gripper.append(currTime)
                currTime += float(1.0/self.rate)
            dataLine_v.setData(self.time_axis_ee, self.ee_velocities)
            dataLine_g.setData(self.time_axis_gripper, self.gripper_velocities)
            self.timeStep.sleep()

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
            
            