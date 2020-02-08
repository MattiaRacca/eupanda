#!/usr/bin/python
from __future__ import division

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QPushButton, QHBoxLayout, QVBoxLayout, QScrollArea, \
QSizePolicy, QGroupBox, QApplication, QStackedWidget, QSlider, QGridLayout, QTabWidget, QLineEdit, QMessageBox
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QSize, QThreadPool, pyqtProperty, QPropertyAnimation
from PyQt5.QtGui import QColor, QPalette, QPixmap, QCursor, QFont, QIcon
import qt_range_slider.qtRangeSlider as qtRangeSlider
from panda_gui.gui_elements import QExpandingPushButton, QVerticalLine, FixNumberTicksSlider, QHorizontalLine

import rospkg
import rospy
from std_msgs.msg import Int32
from franka_control.msg import ErrorRecoveryActionGoal

from panda_eup.program_interpreter import PandaProgramInterpreter
import panda_eup.panda_primitive as pp
from panda_eup.pbd_interface import PandaPBDInterface

import pyttsx3

import os
import traceback
import sys
import pickle
from functools import partial
from enum import Enum
from datetime import datetime
import time

# Size of Primitive Widget
PRIMITIVE_WIDTH = 100
PRIMITIVE_HEIGHT = 120

# Spacings for the Program Widget
H_SPACING = 20
V_SPACING = 30

# Minimum number of visible primitives on screen
MIN_PRIMITIVE = 5

# Color Palettes
gray_palette = QPalette()
white_palette = QPalette()
executed_primitive_palette = QPalette()
error_palette = QPalette()
gray_palette.setColor(QPalette.Background, QColor("gainsboro"))
white_palette.setColor(QPalette.Background, QColor("ghostwhite"))
executed_primitive_palette.setColor(QPalette.Background, QColor("cornflowerblue"))
error_palette.setColor(QPalette.Background, QColor("firebrick"))


class EUPStateMachine(Enum):
    STARTUP = 0
    STARTUP_BUSY = 1
    OPERATIONAL = 2
    BUSY = 3
    STARTUP_ERROR = 4
    EXECUTION_ERROR = 5
    REVERTING_ERROR = 6


class EUPWidget(QWidget):
    # static variables
    font = QFont()
    font.setBold(True)
    font.setPointSize(12)
    size_policy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)

    updateGUI = pyqtSignal()
    programGUIUpdate = pyqtSignal()
    robotStateUpdate = pyqtSignal(pp.PandaRobotStatus)
    tuningGUIUpdate = pyqtSignal(object)
    tuningAccepted = pyqtSignal(bool, type, str)
    rangeAccepted = pyqtSignal(bool, type, str)

    def __init__(self, title='EUP Widget'):
        super(EUPWidget, self).__init__()
        self.setWindowTitle(title)

        # Starting timestamp, for logs name and logging wallclock time
        self.starting_timestamp = time.time()
        self.tuning_timeseries = []  # wallclock time of all primitive tunings
        self.execution_timeseries = []  # wallclock time of all primitive executions

        # Creating the interpreter and loading the program
        robotless_debug = rospy.get_param('/robotless_debug') if rospy.has_param('/robotless_debug') else False
        self.interpreter = PandaProgramInterpreter(robotless_debug=robotless_debug)
        self.interface = PandaPBDInterface(robotless_debug=robotless_debug)

        if rospy.has_param('/program_path') and rospy.has_param('/program_name'):
            program_path = rospy.get_param('/program_path')
            program_name = rospy.get_param('/program_name')
        else:
            program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
            program_name = 'program.pkl'
            rospy.logwarn('Could not find rosparam program_path OR the program_name; loading the example program')

        self.interpreter.load_program(pp.load_program_from_file(program_path, program_name))

        # TODO: this reset of history should be an option! it is now here for dealing with old program.pkl
        self.interpreter.loaded_program.reset_primitives_history()

        # Randomizing, Range Sliders and TTS options
        randomize = False
        if rospy.has_param('/randomize_parameters'):
            randomize = rospy.get_param('/randomize_parameters')

        if randomize:
            rospy.loginfo('Going to randomize the primitives parameters... oh dear')
            self.interpreter.loaded_program.randomize_gui_tunable_primitives()

        self.range_sliders = False
        if rospy.has_param('/range_sliders'):
            self.range_sliders = rospy.get_param('/range_sliders')

        self.tts_for_primitives = False
        if rospy.has_param('/tts_for_primitives'):
            self.tts_for_primitives = rospy.get_param('/tts_for_primitives')

        self.tts_engine = pyttsx3.init()
        voices = self.tts_engine.getProperty('voices')
        self.tts_engine.setProperty('voice', voices[16].id)  # American English
        self.tts_engine.setProperty('volume', 0.8)
        self.tts_engine.setProperty('rate', 180)

        # Setting up the state machines
        self.state_machine = EUPStateMachine.STARTUP
        self.last_interface_state = None

        # Thread-pool for the interpreter commands
        self.threadpool = QThreadPool()
        rospy.logdebug("Multi-threading with maximum %d threads" % self.threadpool.maxThreadCount())

        # Initializing the UI
        self.initUI()

        # Subscriber for the interface status
        self.interface_state_subscriber = rospy.Subscriber("/primitive_interface_node/interface_state", Int32,
                                                           self.interface_state_callback)

    def log_loaded_program(self, need_to_log=True, type_of_primitive=None, name_of_parameter='', partial_log=False):
        if rospy.has_param('/program_logging_path') and need_to_log:
            # naming and pathing for the logs
            program_logging_path = rospy.get_param('/program_logging_path')
            date = datetime.fromtimestamp(self.starting_timestamp).strftime('%m%d_%H%M')
            if not os.path.exists(program_logging_path):
                os.makedirs(program_logging_path)
            filename = '{}_partial.pkl' if partial_log else '{}.pkl'

            # log creation and dumping
            log = {}
            log['program'] = self.interpreter.loaded_program
            log['wallclock_time'] = time.time() - self.starting_timestamp
            log['tuning_timeseries'] = self.tuning_timeseries
            log['execution_timeseries'] = self.execution_timeseries

            with open(os.path.join(os.path.expanduser(program_logging_path), filename.format(date)), 'wb') as f:
                pickle.dump(log, f)
            rospy.loginfo('Current program saved in {}'.format(program_logging_path))
        else:
            rospy.logwarn('Could not find rosparam program_logging_path; skipped program logging')

    def interface_state_callback(self, msg):
        # callback for when interface status msg is received
        new_interface_status = pp.PandaRobotStatus(msg.data)
        if self.last_interface_state != new_interface_status:
            self.last_interface_state = new_interface_status
            self.updateGUI.emit()

    def initUI(self):
        # Create overall layout
        self.vbox = QVBoxLayout(self)
        self.vbox.setAlignment(Qt.AlignTop)
        self.tabSelection = TabWidget(self)
        self.vbox.addWidget(self.tabSelection)
        # Panda Program Widget on top
        self.panda_program_widget = PandaProgramWidget(self)

        # Parameter tuning frame
        self.panda_tuning_widget = PandaTuningWidget(parent=self, range_sliders=self.range_sliders)

        # Action button & Robot State Widget at the bottom
        self.low_buttons = QWidget()
        self.low_buttons_layout = QHBoxLayout(self.low_buttons)
        self.low_buttons_layout.setAlignment(Qt.AlignCenter)

        self.robot_state_widget = PandaStateWidget(self)

        # Making the push button do something useful (call different versions of execute_interpreter_command
        self.interpreter_command_dict = {}
        self.interpreter_command_dict['go_to_starting_state'] = [QExpandingPushButton("Go to\n start state", self),
                                                                 partial(self.execute_interpreter_command,
                                                                         self.interpreter.go_to_starting_state)]
        self.interpreter_command_dict['execute_one_step'] = [QExpandingPushButton("Execute\n one step", self),
                                                             partial(self.execute_interpreter_command,
                                                                     self.interpreter.execute_one_step)]
        self.interpreter_command_dict['revert_one_step'] = [QExpandingPushButton("Revert\n one step", self),
                                                                 partial(self.execute_interpreter_command,
                                                                         self.interpreter.revert_one_step)]
        self.interpreter_command_dict['go_to_current_primitive_preconditions'] = \
            [QExpandingPushButton("Recover from error\n on current primitive", self),
             partial(self.execute_interpreter_command, self.interpreter.go_to_current_primitive_preconditions)]
        self.interpreter_command_dict['execute_rest_of_program'] = [QExpandingPushButton("Execute rest\n of program", self),
                                                                 partial(self.execute_interpreter_command,
                                                                         self.interpreter.execute_rest_of_program)]
        self.interpreter_command_dict['revert_to_beginning_of_program'] = [QExpandingPushButton("Revert to\n beginning", self),
                                                                 partial(self.execute_interpreter_command,
                                                                         self.interpreter.revert_to_beginning_of_program
                                                                         )]

        # Give the partials a __name__ attribute, used in the execute_interpreter_command function
        for key, value in self.interpreter_command_dict.items():
            value[1].__name__ = key

        self.low_buttons_layout.addWidget(self.robot_state_widget)
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['go_to_starting_state'][0])
        self.low_buttons_layout.addWidget(QVerticalLine())
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['execute_one_step'][0])
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['revert_one_step'][0])
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['go_to_current_primitive_preconditions'][0])
        self.low_buttons_layout.addWidget(QVerticalLine())
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['execute_rest_of_program'][0])
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['revert_to_beginning_of_program'][0])

        # PushButtons events handling
        for key, value in self.interpreter_command_dict.items():
            value[0].clicked.connect(value[1])
            value[0].setEnabled(key is 'go_to_starting_state')
            value[0].setVisible(key is not 'go_to_current_primitive_preconditions')
            value[0].setFont(EUPWidget.font)

        # Put everything together
        # self.vbox.addWidget(self.panda_program_widget)
        # self.vbox.addWidget(self.panda_tuning_widget)
        # self.vbox.addWidget(self.low_buttons)
        # self.vbox.addWidget(self.tabs)
        self.tabSelection.runProgramTab.layout.addWidget(self.panda_program_widget)
        self.tabSelection.runProgramTab.layout.addWidget(self.panda_tuning_widget)
        self.tabSelection.runProgramTab.layout.addWidget(self.low_buttons)

        self.program_creation_widget = PandaProgramWidget(self)
        self.program_creation_widget.clear()
        self.program_creation_buttons = ProgramCreationButtons(self)
        self.lowerProgramMenu = LowerProgramMenu(self)

        self.tabSelection.createProgramTab.layout.addWidget(self.program_creation_widget)
        self.tabSelection.createProgramTab.layout.addWidget(self.program_creation_buttons)
        self.tabSelection.createProgramTab.layout.addWidget(QHorizontalLine())
        self.tabSelection.createProgramTab.layout.addWidget(self.lowerProgramMenu)

        self.addPrimitiveButtonActions()
        self.addProgramUtilityActions()
        self.addControlButtonActions()

        # Connect update signals
        self.tuningAccepted.connect(partial(self.log_loaded_program, partial_log=True))  # triggers partial logging after parameter tuning
        self.rangeAccepted.connect(partial(self.log_loaded_program, partial_log=True))  # triggers partial logging after range update
        self.updateGUI.connect(self.updatePandaWidgets)  # overall GUI update, triggers the update below
        self.programGUIUpdate.connect(self.panda_program_widget.updateWidget)  # program widget update
        self.robotStateUpdate.connect(self.robot_state_widget.updateWidget)  # robot state widget update
        self.tuningGUIUpdate.connect(self.panda_tuning_widget.updateWidget)  # tuning widget update

        self.updatePandaWidgets()

    def updateCurrentPrimitive(self):
        if self.state_machine == EUPStateMachine.OPERATIONAL:
            ready_primitive = None
            try:
                ready_primitive = self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index)
                rospy.loginfo('Attempting to tune {}'.format(str(ready_primitive)))
            except pp.PandaProgramException:
                pass

            if ready_primitive is not None:
                tuning_targets = self.panda_tuning_widget.stacks[type(ready_primitive)].current_tuning
                range_tuning_targets = self.panda_tuning_widget.stacks[type(ready_primitive)].range_tuning
                something_tuned = False
                for key, value in tuning_targets.items():
                    tuned = self.interpreter.loaded_program.update_nth_primitive_parameter(
                        self.interpreter.next_primitive_index, key, value)
                    rospy.loginfo('Tuning parameter {} of a {} primitive: {}'.format(key, \
                                                                                     type(ready_primitive), \
                                                                                     str(tuned)))
                    self.tuningAccepted.emit(tuned, type(ready_primitive), key)
                    something_tuned = something_tuned or tuned
                if something_tuned:
                    self.tuning_timeseries.append(time.time())

                for key, value in range_tuning_targets.items():
                    self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index).\
                        update_parameter_range(key, value)
                    rospy.loginfo('Saving range {} of a {} primitive: {}'.format(key, \
                                                                                 type(ready_primitive), \
                                                                                 str(True)))
                    self.rangeAccepted.emit(True, type(ready_primitive), key)

        else:
            rospy.logerr('Are you tuning when you should not?')

    def updatePandaWidgets(self):
        rospy.loginfo('{} | {}'.format(self.last_interface_state, self.state_machine))

        self.programGUIUpdate.emit()
        if self.last_interface_state is not None:
            self.robotStateUpdate.emit(self.last_interface_state)

        ready_primitive = None
        try:
            ready_primitive = self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index)
        except pp.PandaProgramException:
            pass

        self.tuningGUIUpdate.emit(ready_primitive)
        QApplication.restoreOverrideCursor()

        #self.checkForInitialization()
        self.checkEUPState()
        self.checkForRelaxedAndFrozen()
        self.preventMultipleApplyForceFingers()
        self.disableSaveAndReset()
        self.lowerProgramMenu.updateControlStateLabels(self.interface)

        '''
        Notes:
        - Should initialization button be disabled after program has been initialized?
        - Since insert X -methods call freeze() if the robot is relaxed, do we only check for self.relaxed?
        - Is there ever a situation where you would want more than one UserSync in a row? Maybe disable UserSync if previous primitive is UserSync
        - When self.relaxed == True, do we disable all relax related buttons?
        - In addition to robot state, should we show the state or self.relaxed and self.frozen on the GUI?
        - Do we need a separate indicator for relaxed fingers?
        - Need clarification on freeze and relaxed states
        '''

        if self.last_interface_state == pp.PandaRobotStatus.ERROR or \
                self.last_interface_state == pp.PandaRobotStatus.BUSY:
            for key, value in self.interpreter_command_dict.items():
                value[0].setEnabled(False)
            self.panda_tuning_widget.setEnabled(False)
        else:
            if self.state_machine == EUPStateMachine.STARTUP:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_starting_state')
                self.panda_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.OPERATIONAL:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is not 'go_to_starting_state')
                    value[0].setVisible(key is not 'go_to_current_primitive_preconditions')
                self.panda_tuning_widget.setEnabled(True)

                # last primitive executed, disable execute buttons
                if self.interpreter.next_primitive_index == self.interpreter.loaded_program.get_program_length():
                    self.interpreter_command_dict['execute_one_step'][0].setEnabled(False)
                    self.interpreter_command_dict['execute_rest_of_program'][0].setEnabled(False)
                    self.interpreter_command_dict['go_to_starting_state'][0].setEnabled(True)
                    self.panda_tuning_widget.setEnabled(False)

                # we are at start, disable revert buttons
                if self.interpreter.next_primitive_index <= 0:
                    self.interpreter_command_dict['revert_one_step'][0].setEnabled(False)
                    self.interpreter_command_dict['revert_to_beginning_of_program'][0].setEnabled(False)

            elif self.state_machine == EUPStateMachine.STARTUP_BUSY or self.state_machine == EUPStateMachine.BUSY:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(False)
                self.panda_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.STARTUP_ERROR:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_starting_state')
                self.panda_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.EXECUTION_ERROR:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_current_primitive_preconditions')
                    value[0].setVisible(key is not 'revert_one_step')
                    if key == 'go_to_current_primitive_preconditions':
                        value[0].setVisible(True)
                self.panda_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.REVERTING_ERROR:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_current_primitive_preconditions')
                    value[0].setVisible(key is not 'revert_one_step')
                    if key == 'go_to_current_primitive_preconditions':
                        value[0].setVisible(True)
                self.panda_tuning_widget.setEnabled(False)

    def execute_interpreter_command(self, command):
        # Disable lower buttons
        for key, value in self.interpreter_command_dict.items():
            value[0].setDisabled(True)

        if (command.__name__ == 'execute_one_step' or \
            command.__name__ == 'execute_rest_of_program'):
            self.updateCurrentPrimitive()

        if command.__name__ == 'execute_one_step':
            self.execution_timeseries.append(time.time())

        if self.state_machine == EUPStateMachine.STARTUP:
            self.state_machine = EUPStateMachine.STARTUP_BUSY

        if self.state_machine == EUPStateMachine.OPERATIONAL:
            self.state_machine = EUPStateMachine.BUSY

        worker = Worker(command) # Any other args, kwargs are passed to the run function
        worker.signals.result.connect(self.reapInterpreterResults)
        worker.signals.finished.connect(self.announceWorkerDeath)
        worker.signals.progress.connect(self.actOnWorkerUpdate)

        QApplication.setOverrideCursor(QCursor(Qt.WaitCursor)) # TODO: Weird bug makes it work only once...

        self.threadpool.start(worker)

    def reapInterpreterResults(self, success):
        rospy.logdebug("Intepreter result: " + str(success))
        if self.state_machine == EUPStateMachine.STARTUP_BUSY:
            self.state_machine = EUPStateMachine.OPERATIONAL if success else EUPStateMachine.STARTUP_ERROR
        if self.state_machine == EUPStateMachine.BUSY:
            self.state_machine = EUPStateMachine.OPERATIONAL if success else EUPStateMachine.EXECUTION_ERROR
            if self.tts_for_primitives and self.interpreter.last_primitive_attempted is not None:
                if type(self.interpreter.last_primitive_attempted) is pp.UserSync:
                    sentence = self.interpreter.last_primitive_attempted.result_message[success]
                    self.tts_engine.say(sentence)
                    self.tts_engine.runAndWait()
            # if the command failed but the primitive in error is the previous one, I was reverting
            try:
                reverting_check = self.interpreter.loaded_program.get_nth_primitive(
                    self.interpreter.next_primitive_index - 1)
                rospy.logdebug('REVERTING CHECK: {}'.format(reverting_check))
                if not success and reverting_check.status == pp.PandaPrimitiveStatus.ERROR:
                    self.state_machine = EUPStateMachine.REVERTING_ERROR
            except pp.PandaProgramException:
                pass
        if self.state_machine == EUPStateMachine.STARTUP_ERROR and success:
            self.state_machine = EUPStateMachine.STARTUP
        if (self.state_machine == EUPStateMachine.EXECUTION_ERROR or
            self.state_machine == EUPStateMachine.REVERTING_ERROR) and success:
            self.state_machine = EUPStateMachine.OPERATIONAL    
        self.updatePandaWidgets()

    def addControlButtonActions(self):
        controlActions = {
            self.program_creation_buttons.controlButtons[0]: self.interface.initialize_program,
            self.program_creation_buttons.controlButtons[1]: self.interface.freeze,
            self.program_creation_buttons.controlButtons[2]: self.interface.relax,
            self.program_creation_buttons.controlButtons[3]: self.interface.relax_finger,
            self.program_creation_buttons.controlButtons[4]: self.interface.relax_only_arm,
            self.program_creation_buttons.controlButtons[5]: self.interface.relax_only_wrist
        } 
        for k,v in controlActions.items():
            k.pressed.connect(v) 
            k.pressed.connect(self.updatePandaWidgets)  

    def addPrimitiveButtonActions(self):
        primitiveActions = {
            self.program_creation_buttons.primitiveButtons[0]: partial(self.addPrimitive, pp.MoveToEE(), self.interface.insert_move_to_ee),
            self.program_creation_buttons.primitiveButtons[1]: partial(self.addPrimitive, pp.MoveToContact(), self.interface.insert_move_to_contact),
            self.program_creation_buttons.primitiveButtons[2]: partial(self.addPrimitive, pp.UserSync(), self.interface.insert_user_sync),
            self.program_creation_buttons.primitiveButtons[3]: partial(self.addPrimitive, pp.MoveFingers(), self.interface.insert_move_fingers),
            self.program_creation_buttons.primitiveButtons[4]: partial(self.addPrimitive, pp.ApplyForceFingers(), self.interface.insert_apply_force_fingers)
        }
        for k,v in primitiveActions.items():
            k.pressed.connect(v)

    def addPrimitive(self, primitive, fn):
        fn()
        self.program_creation_widget.addPrimitiveWidget(primitive, interpreter=self.interpreter)
        self.program_creation_widget.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH)*self.interface.program.get_program_length(),
                                        V_SPACING + PRIMITIVE_HEIGHT)
        self.program_creation_widget.updateWidget()
        self.updatePandaWidgets()

    def saveProgram(self):
        inputField = self.lowerProgramMenu.inputField
        filename =  inputField.text()
        filePath = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
        if filename == '':
            self.interface.program.dump_to_file(filePath, "testprogram.pkl")
        else:
             self.interface.program.dump_to_file(filePath, str(filename) + '.pkl')   
        inputField.clear()
        buttonReply = QMessageBox.question(self, 'PyQt5 message', "Load this program for execution?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if buttonReply == QMessageBox.Yes:
            self.loadNewProgram()
        

    def loadNewProgram(self):
        self.interpreter.load_program(self.interface.program)  
        self.updatePandaWidgets()
        self.panda_program_widget.clear()
        for primitive in self.interface.program.primitives:
            self.panda_program_widget.addPrimitiveWidget(primitive, self.interpreter)
        self.panda_program_widget.update()    
        

    def addProgramUtilityActions(self):
        self.lowerProgramMenu.saveButton.pressed.connect(self.saveProgram)
        self.lowerProgramMenu.resetButton.pressed.connect(partial(self.program_creation_widget.clear, self.interface))
        self.lowerProgramMenu.resetButton.pressed.connect(self.updatePandaWidgets)      

    def checkForInitialization(self):
        for controlButton in self.program_creation_buttons.controlButtons[1:]:
            controlButton.setEnabled(self.interface.program.initialized)

        for primitiveButton in self.program_creation_buttons.primitiveButtons:
            primitiveButton.setEnabled(self.interface.program.initialized) 

    def checkForRelaxedAndFrozen(self):
        if self.interface.frozen == False:
            for primitiveButton in self.program_creation_buttons.primitiveButtons:
                primitiveButton.setEnabled(False)

        if self.interface.relaxed:
            self.program_creation_buttons.controlButtons[2].setEnabled(False)

        if self.interface.frozen:
            self.program_creation_buttons.controlButtons[1].setEnabled(False)                    

    def checkEUPState(self):
        state = self.state_machine in [EUPStateMachine.STARTUP, EUPStateMachine.OPERATIONAL]
        if state == False:
            rospy.loginfo("Program creation buttons disabled while the state machine is busy or in error state")
        else:
            rospy.loginfo("Program creation buttons enabled")    
        for controlButton in self.program_creation_buttons.controlButtons:
            controlButton.setEnabled(state)

        for primitiveButton in self.program_creation_buttons.primitiveButtons:
            primitiveButton.setEnabled(state)
        self.checkForInitialization()

    def preventMultipleApplyForceFingers(self):
        if len(self.interface.program.primitives) > 0:
            lastPrimitive = self.interface.program.primitives[-1]
            if lastPrimitive.__class__.__name__ == "ApplyForceFingers":
                self.program_creation_buttons.primitiveButtons[4].setEnabled(False)

    def disableSaveAndReset(self):
        if len(self.interface.program.primitives) == 0:
            self.lowerProgramMenu.saveButton.setEnabled(False)
            self.lowerProgramMenu.resetButton.setEnabled(False)      
        else:
            self.lowerProgramMenu.saveButton.setEnabled(True)
            self.lowerProgramMenu.resetButton.setEnabled(True) 



    def announceWorkerDeath(self):
        rospy.logdebug("RIP Worker!")

    def actOnWorkerUpdate(self, progress):
        self.updatePandaWidgets()

    def sizeHint(self):
        return QSize(1280, 720)

    def minimumSizeHint(self):
        return QSize(800, 600)


class TabWidget(QWidget):

    def __init__(self, parent):
        super(TabWidget, self).__init__(parent)
        self.layout = QVBoxLayout()
        self.tabWidget = QTabWidget()
        self.runProgramTab = QWidget()
        self.createProgramTab = QWidget()

        self.tabWidget.addTab(self.runProgramTab,"Run Programs")
        self.tabWidget.addTab(self.createProgramTab,"Create Programs")

        self.runProgramTab.layout = QVBoxLayout(self)
        self.runProgramTab.layout.setAlignment(Qt.AlignTop)
        self.createProgramTab.layout = QVBoxLayout(self)
        self.createProgramTab.layout.setAlignment(Qt.AlignTop)

        self.runProgramTab.setLayout(self.runProgramTab.layout)
        self.createProgramTab.setLayout(self.createProgramTab.layout)

        self.layout.addWidget(self.tabWidget)
        self.setLayout(self.layout)

class ProgramCreationButtons(QWidget):
    sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)

    def __init__(self, parent):
        super(ProgramCreationButtons, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.setSizePolicy(self.sizePolicy)
        self.layout = QHBoxLayout(self)
        self.addPrimitiveButtons()
        self.layout.addWidget(QVerticalLine())
        self.addControlButtons()

    def addPrimitiveButtons(self):
        self.primitiveButtonAreaWidget = QWidget(self)
        self.primitiveButtons = []
        self.primitiveVerticalLayout = QVBoxLayout(self.primitiveButtonAreaWidget)
        self.primitiveVerticalLayout.setAlignment(Qt.AlignTop)
        label = QLabel("Add primitive for program")
        label.setFont(EUPWidget.font)
        self.primitiveVerticalLayout.addWidget(label)
        self.primitiveButtonRowWidget = QWidget(self)
        self.primitiveButtonLayout = QHBoxLayout(self.primitiveButtonRowWidget)
        self.primitiveButtonLayout.setAlignment(Qt.AlignLeft)
        primitives = [pp.MoveToEE(), pp.MoveToContact(), pp.UserSync(), pp.MoveFingers(), pp.ApplyForceFingers()]
        for primitive in primitives:
            button = QPushButton('', self)
            imagePath = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources', primitive.__class__.__name__ + '.png')
            button.setIcon(QIcon(imagePath))
            button.setIconSize(QSize(80,80))
            self.primitiveButtonLayout.addWidget(button)
            self.primitiveButtons.append(button)
        self.primitiveVerticalLayout.addWidget(self.primitiveButtonRowWidget)    
        self.layout.addWidget(self.primitiveButtonAreaWidget)    

    def addControlButtons(self):
        self.controlButtonWidget = QWidget(self)
        self.controlButtons = []
        self.controlButtonLayout = QGridLayout(self.controlButtonWidget)
        self.controlButtonLayout.setAlignment(Qt.AlignRight)
        labels = ["Initialize\nprogram", "Freeze", "Relax", "Relax\nfingers", "Relax\nonly arm", "Relax\nonly wrist"]
        for i in range(len(labels)):
            button = QExpandingPushButton(labels[i], self)
            button.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
            button.setFont(EUPWidget.font)
            self.controlButtonLayout.addWidget(button, i/3, i % 3)
            self.controlButtons.append(button)
        self.layout.addWidget(self.controlButtonWidget)            

class LowerProgramMenu(QWidget):

    def __init__(self, parent):
        super(LowerProgramMenu, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.layout = QHBoxLayout(self)
        self.layout.setAlignment(Qt.AlignLeft)    
        self.stateWidget = PandaStateWidget(self)
        self.layout.addWidget(self.stateWidget)
        self.addControlStateLabels()
        self.layout.addWidget(QVerticalLine())
        self.addProgramUtilities()

    def addControlStateLabels(self):
        font=QFont()
        font.setBold(True)
        font.setPointSize(12)

        self.controlStateWidget = QWidget()
        self.controlStateLayout = QVBoxLayout(self.controlStateWidget)

        self.frozenLabel = QLabel("Frozen")
        self.frozenLabel.setFont(font)
        self.frozenLabel.setAlignment(Qt.AlignCenter)

        self.frozenLabelValue = QLabel("No")
        self.frozenLabelValue.setFont(font)
        self.frozenLabelValue.setStyleSheet('color: firebrick')
        self.frozenLabelValue.setAlignment(Qt.AlignCenter)

        self.relaxedLabel = QLabel("Relaxed")
        self.relaxedLabel.setFont(font)
        self.relaxedLabel.setAlignment(Qt.AlignCenter
        )
        self.relaxedLabelValue = QLabel("No")
        self.relaxedLabelValue.setStyleSheet('color: firebrick')
        self.relaxedLabelValue.setFont(font)
        self.relaxedLabelValue.setAlignment(Qt.AlignCenter)

        self.controlStateLayout.addWidget(self.frozenLabel)
        self.controlStateLayout.addWidget(self.frozenLabelValue)
        self.controlStateLayout.addStretch()
        self.controlStateLayout.addWidget(self.relaxedLabel)
        self.controlStateLayout.addWidget(self.relaxedLabelValue)

        self.layout.addWidget(self.controlStateWidget) 

    def updateControlStateLabels(self, interface):
        if interface.frozen:
            self.frozenLabelValue.setText("Yes")
            self.frozenLabelValue.setStyleSheet('color: lightseagreen')
        else:
            self.frozenLabelValue.setText("No")
            self.frozenLabelValue.setStyleSheet('color: firebrick')

        if interface.relaxed:
            self.relaxedLabelValue.setText("Yes")
            self.relaxedLabelValue.setStyleSheet('color: lightseagreen')
        else:
            self.relaxedLabelValue.setText("No")
            self.relaxedLabelValue.setStyleSheet('color: firebrick')



    def addProgramUtilities(self):
        self.programUtilities = QWidget()
        self.utilitiesLayout = QHBoxLayout(self.programUtilities)
        self.saveButton = QPushButton("Save Program")
        self.inputField = QLineEdit()
        self.inputField.setPlaceholderText("Enter name for saved file")
        self.resetButton = QPushButton("Reset Program")
        self.saveButton.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.resetButton.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        #self.saveButton.setFont(EUPWidget.font)
        #self.resetButton.setFont(EUPWidget.font)
        self.utilitiesLayout.addWidget(self.saveButton)
        self.utilitiesLayout.addWidget(self.inputField)
        self.utilitiesLayout.addWidget(self.resetButton) 
        self.layout.addWidget(self.programUtilities)   
        

class PandaProgramWidget(QGroupBox):
    sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)

    def __init__(self, parent):
        super(PandaProgramWidget, self).__init__(parent.interpreter.loaded_program.name, parent)
        self.initUI()

    def initUI(self):
        self.primitive_widget_list = []

        # Create layout for program widget and add Scroll Area
        self.widget_layout = QHBoxLayout(self)
        self.program_scroll_area = QScrollArea(self)
        self.widget_layout.addWidget(self.program_scroll_area)
        self.program_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.program_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Set the policy to expand on horizontal axis
        self.setSizePolicy(PandaProgramWidget.sizePolicy)

        # Create container and layout for PandaPrimitiveWidgets
        self.program_widget = QWidget(self)
        self.program_widget_layout = QHBoxLayout(self.program_widget)
        self.program_widget_layout.setAlignment(Qt.AlignLeft)
        self.program_widget.setSizePolicy(PandaProgramWidget.sizePolicy)

        # Color the Program area
        self.setAutoFillBackground(True)
        self.setPalette(gray_palette)
        self.program_widget.setAutoFillBackground(True)
        self.program_widget.setPalette(gray_palette)

        # Add scrolling area
        program = self.parent().interpreter.loaded_program
        self.program_scroll_area.setWidget(self.program_widget)
        self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH)*program.get_program_length(),
                                        V_SPACING + PRIMITIVE_HEIGHT)

        # Add primitive Widgets
        for primitive in program.primitives:
            self.addPrimitiveWidget(primitive, self.parent().interpreter)

    def addPrimitiveWidget(self, primitive, interpreter):
        primitive_widget = PandaPrimitiveWidget(self.program_widget, primitive)
        self.primitive_widget_list.append(primitive_widget)
        self.program_widget_layout.addWidget(primitive_widget)

        program_length = interpreter.loaded_program.get_program_length()
        if self.program_widget.width() < (H_SPACING + PRIMITIVE_WIDTH)*program_length:
            self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH)*program_length, V_SPACING +
                                            PRIMITIVE_HEIGHT)

    def updateWidget(self):
        for i, primitive_widget in enumerate(self.primitive_widget_list):
            primitive_widget.updateWidget()
        self.update()

    def clear(self, interface=None):
        for primitive_widget in self.primitive_widget_list:
            primitive_widget.setParent(None)
        self.primitive_widget_list = []
        if interface != None:
            interface.program = pp.PandaProgram("A Panda Program")
        self.update()        

    def sizeHint(self):
        return QSize((H_SPACING+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, V_SPACING*3 + PRIMITIVE_HEIGHT)

    def minimumSizeHint(self):
        return QSize((H_SPACING+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, V_SPACING*3 + PRIMITIVE_HEIGHT)


class PandaStateWidget(QGroupBox):
    # static variables
    sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    status_color = {
        pp.PandaRobotStatus.READY: 'lightseagreen',
        pp.PandaRobotStatus.BUSY: 'gold',
        pp.PandaRobotStatus.ERROR: 'firebrick'
    }

    font=QFont()
    font.setBold(True)
    font.setPointSize(10)

    def __init__(self, parent):
        super(PandaStateWidget, self).__init__('Robot State', parent)
        self.error_recover_publisher = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal,
                                                       queue_size=10)
        self.initUI()

    def initUI(self):
        self.status_label = QLabel('Undefined')

        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(PandaStateWidget.font)

        self.recover_button = QPushButton('Recover \nfrom Error')
        self.recover_button.setEnabled(False)
        self.recover_button.clicked.connect(self.sendErrorRecover)

        layout = QVBoxLayout(self)
        layout.addWidget(self.status_label)
        layout.addWidget(self.recover_button)

        self.setSizePolicy(PandaStateWidget.sizePolicy)
        self.setAlignment(Qt.AlignCenter)

        self.setLayout(layout)

    def sizeHint(self):
        return QSize(150, PRIMITIVE_HEIGHT)

    def sendErrorRecover(self):
        msg = ErrorRecoveryActionGoal()
        self.error_recover_publisher.publish(msg)

    def updateWidget(self, status):
        self.status_label.setText(status.name)
        try:
            self.status_label.setStyleSheet('background-color: ' + PandaStateWidget.status_color[status])
        except KeyError:
            pass

        self.recover_button.setEnabled(status == pp.PandaRobotStatus.ERROR)

        self.update()


class PandaPrimitiveWidget(QFrame):
    # static variables
    status_label_dict = {
        pp.PandaPrimitiveStatus.NEUTRAL:'',
        pp.PandaPrimitiveStatus.READY:pp.PandaPrimitiveStatus.READY.name,
        pp.PandaPrimitiveStatus.ERROR:pp.PandaPrimitiveStatus.ERROR.name,
        pp.PandaPrimitiveStatus.EXECUTING:pp.PandaPrimitiveStatus.EXECUTING.name,
        pp.PandaPrimitiveStatus.REVERTING:pp.PandaPrimitiveStatus.REVERTING.name,
        pp.PandaPrimitiveStatus.EXECUTED:''
    }
    font=QFont()
    font.setBold(True)
    font.setPointSize(10)

    sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def __init__(self, parent, panda_primitive):
        super(PandaPrimitiveWidget, self).__init__(parent)
        self.initUI(panda_primitive)

    def initUI(self, panda_primitive):
        # Create widget subcomponents
        self.primitive = panda_primitive
        self.primitive_label = QLabel()

        self.status_label = QLabel(str(PandaPrimitiveWidget.status_label_dict[panda_primitive.status]))
        self.status_label.setAlignment(Qt.AlignCenter)

        # Fetch fitting icon for primitive
        primitive_icon_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources',
                                           panda_primitive.__class__.__name__ + '.png')

        primitive_image = QPixmap(primitive_icon_path)
        self.primitive_label.setPixmap(primitive_image)

        # Add vertical layout
        layout = QVBoxLayout(self)
        layout.addWidget(self.primitive_label)
        layout.addWidget(self.status_label)

        self.setSizePolicy(PandaPrimitiveWidget.sizePolicy)

        # Beautify QFrame and Color
        self.setFrameShape(QFrame.Panel)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(2)

        self.status_label.setFont(PandaPrimitiveWidget.font)

        # Animation
        self.animation = QPropertyAnimation(self, 'background_color')
        self.animation.setDuration(2000) # in ms
        self.animation.setLoopCount(-1)
        self.animation.setStartValue(QColor('ghostwhite'))
        self.animation.setEndValue(QColor('ghostwhite'))
        self.animation.setKeyValueAt(0.5, QColor('cornflowerblue'))

        self.setAutoFillBackground(True)
        self.setPalette(gray_palette)

    def get_background_color(self):
        return self.palette().color(QPalette.Background)

    def set_background_color(self, color):
        palette = QPalette()
        palette.setColor(QPalette.Background, color)
        self.setPalette(palette)

    background_color = pyqtProperty(QColor, get_background_color, set_background_color)

    def sizeHint(self):
        return QSize(PRIMITIVE_WIDTH, PRIMITIVE_HEIGHT)

    def updateWidget(self):
        if self.primitive.status == pp.PandaPrimitiveStatus.EXECUTING or \
                self.primitive.status == pp.PandaPrimitiveStatus.REVERTING:
            self.animation.start()
        elif self.primitive.status == pp.PandaPrimitiveStatus.ERROR:
            self.animation.stop()
            self.setPalette(error_palette)
        else:
            self.animation.stop()
            if self.primitive.status == pp.PandaPrimitiveStatus.EXECUTED:
                self.setPalette(executed_primitive_palette)
            elif self.primitive.status == pp.PandaPrimitiveStatus.READY:
                self.setPalette(white_palette)
            else:
                self.setPalette(gray_palette)

        self.status_label.setText(str(PandaPrimitiveWidget.status_label_dict[self.primitive.status]))
        self.update()


class PandaTuningWidget(QStackedWidget):
    # static variables
    sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
    tunable_primitives = [pp.MoveToEE, pp.MoveToContact, pp.MoveFingers, pp.ApplyForceFingers, pp.UserSync]

    def __init__(self, parent, range_sliders=False):
        super(PandaTuningWidget, self).__init__(parent)
        self.range_sliders = range_sliders
        self.initUI()

    def initUI(self):
        self.stacks = {}
        self.stacks[None] = PandaTuningPage(self, None)
        self.addWidget(self.stacks[None])

        for primitive_type in PandaTuningWidget.tunable_primitives:
            self.stacks[primitive_type] = PandaTuningPage(self, primitive_type, range_slider_enabled=self.range_sliders)
            self.addWidget(self.stacks[primitive_type])
            self.parent().tuningAccepted.connect(self.stacks[primitive_type].updateAfterTuningAccepted)
            self.parent().rangeAccepted.connect(self.stacks[primitive_type].updateAfterRangeAccepted)

        self.setCurrentIndex(0)
        self.setSizePolicy(PandaTuningWidget.sizePolicy)

    def updateWidget(self, primitive):
        if primitive is None:
            self.setCurrentIndex(0)
        else:
            self.setCurrentIndex(PandaTuningWidget.tunable_primitives.index(primitive.__class__) + 1)
            self.stacks[primitive.__class__].updatePageFromPritimive(primitive)
        self.update()


class PandaTuningPage(QFrame):
    def __init__(self, parent, primitive_type, range_slider_enabled=False):
        super(PandaTuningPage, self).__init__(parent)
        self.primitive_type = primitive_type
        self.range_slider_enabled = range_slider_enabled
        self.initUI()
        self.current_tuning = {}
        self.range_tuning = {}

    def initUI(self):
        layout = QVBoxLayout(self)
        self.sliders = {}
        if self.primitive_type is not None:
            for param in self.primitive_type.gui_tunable_parameters:
                self.sliders[param] = CurrentValueShowingSlider(self, param,
                                                                self.primitive_type.gui_tunable_parameter_units[param],
                                                                self.primitive_type.gui_tunable_parameter_ranges[param],
                                                                range_slider_enabled=self.range_slider_enabled)
                self.sliders[param].valueChanged.connect(partial(self.setParameterTuning, param))
                self.sliders[param].rangeChanged.connect(partial(self.setRangeTuning, param))
                layout.addWidget(self.sliders[param])
        layout.setAlignment(Qt.AlignTop)

    def updatePageFromPritimive(self, primitive):
        if primitive.__class__ is not None:
            for param in primitive.__class__.gui_tunable_parameters:
                self.sliders[param].setValue(getattr(primitive.parameter_container, param))
                self.sliders[param].slider.setStrictBounds(primitive.gui_tunable_parameter_strict_ranges[param])
                if self.range_slider_enabled:
                    current_range = []
                    try:
                        current_range = primitive.parameters_range_history[param][-1]
                    except:
                        current_range = primitive.gui_tunable_parameter_strict_ranges[param]
                    self.sliders[param].range_slider.setValues(current_range)
                    self.sliders[param].range_slider.setStrictRange(primitive.gui_tunable_parameter_strict_ranges[param])

    def setParameterTuning(self, parameter_name, parameter_value):
        self.current_tuning[parameter_name] = parameter_value

    def setRangeTuning(self, parameter_name, min_value, max_value):
        self.range_tuning[parameter_name] = [min_value, max_value]

    def updateAfterTuningAccepted(self, tuned, primitive_type, parameter):
        if self.primitive_type is primitive_type:
            try:
                self.sliders[parameter].receiveValueConfirmation(tuned)
            except KeyError:
                pass
            if tuned:
                del self.current_tuning[parameter]

    def updateAfterRangeAccepted(self, tuned, primitive_type, parameter):
        if self.primitive_type is primitive_type:
            if tuned:
                del self.range_tuning[parameter]


class CurrentValueShowingSlider(QWidget):
    valueChanged = pyqtSignal(float)
    rangeChanged = pyqtSignal(float, float)
    LABEL_WIDTH = 100
    font=QFont()
    font.setBold(True)
    font.setPointSize(11)

    readable_parameter_name = {
        'position_speed': 'Motion Speed',
        'force_threshold': 'Collision Threshold',
        'force': 'Grasp Strength',
        'width': 'Finger Distance'
    }

    def __init__(self, parent, name, measure_unit='', available_range=[0.0, 1.0], range_slider_enabled=False):
        super(CurrentValueShowingSlider, self).__init__(parent)
        self.measure_unit = measure_unit
        self.available_range = available_range
        self.name = name
        self.range_slider_enabled=range_slider_enabled
        self.initUI()

    def initUI(self):
        self.widget_layout = QGridLayout(self)
        n_ticks = 50

        self.slider = FixNumberTicksSlider(self.available_range[0], self.available_range[1], n_ticks, Qt.Horizontal)
        if self.range_slider_enabled:
            min_value = self.available_range[0]
            max_value = self.available_range[1]
            step = (max_value - min_value) / n_ticks  # TODO: this should be a parameter
            self.range_slider = qtRangeSlider.QHRangeSlider(slider_range=[min_value, max_value, step],
                                                            values=[min_value, max_value])

        self.current_value_label = QLabel('???')
        self.stored_value_label = QLabel('???')
        self._current_label = QLabel('Current\n Value')
        self._stored_label = QLabel('Previous\n Value')
        self.name_label = QLabel(CurrentValueShowingSlider.readable_parameter_name[self.name])

        self.current_value_label.setFont(CurrentValueShowingSlider.font)
        self.stored_value_label.setFont(CurrentValueShowingSlider.font)
        self._current_label.setFont(CurrentValueShowingSlider.font)
        self._stored_label.setFont(CurrentValueShowingSlider.font)
        self.name_label.setFont(CurrentValueShowingSlider.font)

        self.current_value_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)
        self.stored_value_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)
        self._current_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)
        self._stored_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)

        self.current_value_label.setAlignment(Qt.AlignCenter)
        self.stored_value_label.setAlignment(Qt.AlignCenter)
        self._current_label.setAlignment(Qt.AlignCenter)
        self._stored_label.setAlignment(Qt.AlignCenter)

        self.widget_layout.addWidget(self.name_label, 0, 0)
        self.widget_layout.addWidget(self.slider, 2, 0)
        self.widget_layout.addWidget(self.current_value_label, 2, 1)
        self.widget_layout.addWidget(self._current_label, 1, 1)
        self.widget_layout.addWidget(self.stored_value_label, 2, 2)
        self.widget_layout.addWidget(self._stored_label, 1, 2)
        if self.range_slider_enabled:
            self.widget_layout.addWidget(self.range_slider, 1, 0)

        self.slider.doubleValueChanged.connect(self.updateLabel)
        self.slider.doubleValueChanged.connect(self.valueChanged.emit)
        if self.range_slider_enabled:
            self.range_slider.rangeChanged.connect(self.rangeChanged.emit)

    def setValue(self, value):
        self.slider.setValue(value)
        self.current_value_label.setText('{:.3f} {}'.format(value, self.measure_unit))
        self.stored_value_label.setText('{:.3f} {}'.format(value, self.measure_unit))

    def updateLabel(self, value):
        self.current_value_label.setText('{:.3f} {}'.format(value, self.measure_unit))
        if self.current_value_label.text() != self.stored_value_label.text():
            self._current_label.setStyleSheet("color: lightseagreen")
            self.current_value_label.setStyleSheet("color: lightseagreen")
        else:
            self._current_label.setStyleSheet("color: black")
            self.current_value_label.setStyleSheet("color: black")

    def receiveValueConfirmation(self, tuned):
        if tuned:
            value = self.slider.value()
            self.stored_value_label.setText('{:.3f} {}'.format(value, self.measure_unit))
            self.updateLabel(value)
        self.update()


class WorkerSignals(QObject):
    '''
    WorkerSignals

    Defines the signals available from a running worker thread.

    Supported signals are:

    finished
        No data

    error
        `tuple` (exctype, value, traceback.format_exc() )

    result
        `object` data returned from processing, anything

    progress
        `int` indicating % progress

    '''
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(object)
    progress = pyqtSignal(int)


class Worker(QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''
    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''

        # Retrieve args/kwargs here; and fire processing using them
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()  # Done
