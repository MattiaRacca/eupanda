#!/usr/bin/python
from __future__ import division

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QPushButton, QHBoxLayout, QVBoxLayout, QScrollArea, \
QSizePolicy, QGroupBox, QApplication, QStackedWidget, QSlider, QGridLayout, QTabWidget, QLineEdit, QMessageBox, QInputDialog
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QSize, QThreadPool, pyqtProperty, QPropertyAnimation
from PyQt5.QtGui import QColor, QPalette, QPixmap, QCursor, QFont, QIcon
import qt_range_slider.qtRangeSlider as qtRangeSlider
from panda_gui.gui_elements import QExpandingPushButton, QVerticalLine, FixNumberTicksSlider, QHorizontalLine
import pyqtgraph as pg

import rospkg
import rospy
from std_msgs.msg import Int32
from franka_control.msg import ErrorRecoveryActionGoal

from panda_eup.program_interpreter import PandaProgramInterpreter
import panda_eup.panda_primitive as pp
from panda_eup.pbd_interface import PandaPBDInterface
from panda_demos.data_recorder import Datarecorder
from panda_demos.segmentation import Segmentation
from panda_pbd.msg import UserSyncGoal, MoveToContactGoal, MoveToEEGoal

import pyttsx3

import os
import traceback
import sys
import pickle
from functools import partial
from enum import Enum
from datetime import datetime
import time
from copy import deepcopy

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
        self.pbd_interface = PandaPBDInterface(robotless_debug=robotless_debug)

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

        # Validation buttons represent the buttons that allow the user to do slight modifications to the program
        # in the tuning-tab, including adding user synchronizations, merging linear motions and converting
        # linear motion to push motion and vice versa
        self.validationButtons = []
        self.validationButtonWidget = QWidget()
        self.validationButtonLayout = QHBoxLayout(self.validationButtonWidget)

        label = QLabel("Add an User Sync\nprimitive to the\ncurrent location")
        label.setFont(EUPWidget.font)
        button = QExpandingPushButton("Add User Sync")
        button.setFont(EUPWidget.font)
        self.validationButtons.append(button)
        self.validationButtonLayout.addWidget(label)
        self.validationButtonLayout.addWidget(button)
        self.validationButtonLayout.addWidget(QVerticalLine())

        label = QLabel("Combine the current\nLinear Motion with\nthe previous one")
        label.setFont(EUPWidget.font)
        button = QExpandingPushButton("Combine")
        button.setFont(EUPWidget.font)
        self.validationButtons.append(button)
        self.validationButtonLayout.addWidget(label)
        self.validationButtonLayout.addWidget(button)
        self.validationButtonLayout.addWidget(QVerticalLine())

        self.motionlabel = QLabel("Convert the current\nLinear Motion to\na Push Motion")
        self.motionlabel.setFont(EUPWidget.font)
        button = QExpandingPushButton("Convert")
        button.setFont(EUPWidget.font)
        self.validationButtons.append(button)
        self.validationButtonLayout.addWidget(self.motionlabel)
        self.validationButtonLayout.addWidget(button)

        # Assign each button to its respective function
        self.validationButtons[0].pressed.connect(self.add_validation_usersync)
        self.validationButtons[1].pressed.connect(self.combine_motion_with_previous)
        self.validationButtons[2].pressed.connect(self.convert_motion)
        
        # Add created widgets to run program tab
        self.tabSelection.runProgramTab.layout.addWidget(self.panda_program_widget)
        self.tabSelection.runProgramTab.layout.addWidget(self.panda_tuning_widget)
        self.tabSelection.runProgramTab.layout.addWidget(QHorizontalLine())
        self.tabSelection.runProgramTab.layout.addWidget(self.validationButtonWidget)
        self.tabSelection.runProgramTab.layout.addWidget(QHorizontalLine())
        self.tabSelection.runProgramTab.layout.addWidget(self.low_buttons)

        # The following additions are for create programs tab
        # Using PandaProgramWidget again, we create a widget thats lists primitives for the program to be created.
        # We also add buttons for program creation and for utilities like saving the program.
        self.program_creation_widget = PandaProgramWidget(self)
        self.program_creation_widget.clear()
        self.program_creation_buttons = ProgramCreationButtons(self)
        self.lowerProgramMenu = LowerProgramMenu(self)

        # These widgets are for the demonstrations tab
        self.demo_program_widget = PandaProgramWidget(self)
        self.demo_program_widget.clear()
        self.demonstrationMenu = DemonstrationMenu(self)
        self.lowerDemoMenu = LowerProgramMenu(self)
        self.demonstrationMenu.lowerDemoMenu = self.lowerDemoMenu
        self.regularizationSlider = CurrentValueShowingSlider(self, "regularization",
                                                              'm',
                                                              [0.01, 0.25],
                                                              range_slider_enabled=self.range_sliders, n_ticks=24)
        self.regularizationSlider.widget_layout.itemAt(4).widget().setVisible(False)
        self.regularizationSlider.widget_layout.itemAt(5).widget().setVisible(False)
        self.regularizationSlider.slider.setValue(0.10)
        self.lowerDemoMenu.saveButton.setText("Save Data")
        self.lowerDemoMenu.resetButton.setVisible(False)
        self.lowerDemoMenu.saveButton.pressed.connect(self.demonstrationMenu.saveData)

        # Add recently created widgets to create programs tab
        self.tabSelection.createProgramTab.layout.addWidget(self.program_creation_widget)
        self.tabSelection.createProgramTab.layout.addWidget(self.program_creation_buttons)
        self.tabSelection.createProgramTab.layout.addWidget(QHorizontalLine())
        self.tabSelection.createProgramTab.layout.addWidget(self.lowerProgramMenu)

        # Add demonstration widgets to their respective tab
        self.tabSelection.demonstrationsTab.layout.addWidget(self.demo_program_widget)
        self.tabSelection.demonstrationsTab.layout.addWidget(self.demonstrationMenu)
        self.tabSelection.demonstrationsTab.layout.addWidget(QHorizontalLine())
        self.tabSelection.demonstrationsTab.layout.addWidget(self.regularizationSlider)
        self.tabSelection.demonstrationsTab.layout.addWidget(QHorizontalLine())
        self.tabSelection.demonstrationsTab.layout.addWidget(self.lowerDemoMenu)

        # Assign actions for each button of create programs tab
        self.addPrimitiveButtonActions()
        self.addProgramUtilityActions()
        self.addControlButtonActions()

        # Connect update signals
        self.tuningAccepted.connect(partial(self.log_loaded_program, partial_log=True))  # triggers partial logging after parameter tuning
        self.rangeAccepted.connect(partial(self.log_loaded_program, partial_log=True))  # triggers partial logging after range update
        self.updateGUI.connect(self.updatePandaWidgets)  # overall GUI update, triggers the update below
        self.programGUIUpdate.connect(self.panda_program_widget.updateWidget)  # program widget update
        self.robotStateUpdate.connect(self.robot_state_widget.updateWidget)  # robot state widget update
        self.robotStateUpdate.connect(self.lowerProgramMenu.stateWidget.updateWidget)
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
                    self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index). \
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

        # EUP State is checked and certain buttons are disabled accordingly
        self.checkEUPState()

        # We disabled frozen- or relaxed-buttons based on which state is active
        self.checkForRelaxedAndFrozen()

        # Disable Finger grasp -primitve if one was added as last primitive (this results in error)
        self.preventMultipleApplyForceFingers()

        # Disable certain buttons if the program is empty such as reseting the program
        self.disableWithEmptyProgram()

        # Labels for the state are updated
        self.lowerProgramMenu.updateControlStateLabels(self.pbd_interface)

        # Disable or Enable validation buttons based on current primitive
        self.updateValidationButtons()

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

        worker = Worker(command)  # Any other args, kwargs are passed to the run function
        worker.signals.result.connect(self.reapInterpreterResults)
        worker.signals.finished.connect(self.announceWorkerDeath)
        worker.signals.progress.connect(self.actOnWorkerUpdate)

        QApplication.setOverrideCursor(QCursor(Qt.WaitCursor))  # TODO: Weird bug makes it work only once...

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
        '''
        These buttons handle the initialization of the robot as well as changnig its state from frozen to relaxed. Here we assign
        an action for each of them.
        '''
        controlActions = {
            self.program_creation_buttons.controlButtons[0]: self.pbd_interface.initialize_program,
            self.program_creation_buttons.controlButtons[1]: self.pbd_interface.freeze,
            self.program_creation_buttons.controlButtons[2]: self.pbd_interface.relax,
            self.program_creation_buttons.controlButtons[3]: self.pbd_interface.relax_finger,
            self.program_creation_buttons.controlButtons[4]: self.pbd_interface.relax_only_arm,
            self.program_creation_buttons.controlButtons[5]: self.pbd_interface.relax_only_wrist
        }
        for k, v in controlActions.items():
            k.pressed.connect(v)
            k.pressed.connect(self.updatePandaWidgets)

    def addPrimitiveButtonActions(self):
        '''
        We assign actions for buttons that are meant for adding new primitives to the program. The button for deleting
        previous primitive is also handled here.
        '''
        primitiveActions = {
            self.program_creation_buttons.primitiveButtons[0]: partial(self.addPrimitive, pp.MoveToEE(), self.pbd_interface.insert_move_to_ee),
            self.program_creation_buttons.primitiveButtons[1]: partial(self.addPrimitive, pp.MoveToContact(), self.pbd_interface.insert_move_to_contact),
            self.program_creation_buttons.primitiveButtons[2]: partial(self.addPrimitive, pp.UserSync(), self.pbd_interface.insert_user_sync),
            self.program_creation_buttons.primitiveButtons[3]: partial(self.addPrimitive, pp.MoveFingers(), self.pbd_interface.insert_move_fingers),
            self.program_creation_buttons.primitiveButtons[4]: partial(self.addPrimitive, pp.ApplyForceFingers(), self.pbd_interface.insert_apply_force_fingers)
        }
        for k, v in primitiveActions.items():
            k.pressed.connect(v)

        self.program_creation_buttons.deleteButton.pressed.connect(self.deletePreviousPrimitive)

    def addPrimitive(self, primitive, fn):
        '''
        Gets primitive and the respective function for adding said primitive to the program as parameters. The primitive is added
        to the program widget, the geometry of the widget is updated and the UI is updated for button state updates.
        '''
        fn()
        self.program_creation_widget.addPrimitiveWidget(primitive, interpreter=self.pbd_interface.interpreter)
        self.program_creation_widget.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH) * self.pbd_interface.program.get_program_length(),
                                        V_SPACING + PRIMITIVE_HEIGHT)
        self.program_creation_widget.updateWidget()
        self.updatePandaWidgets()

    def saveProgram(self):
        '''
        The program is saved to resources-folder using the text that was input into the inputfield. The user is then asked
        whether the recently saved program should be loaded to run program -tab for execution
        '''
        inputField = self.lowerProgramMenu.inputField
        filename = inputField.text()
        filePath = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
        if filename == '':
            self.pbd_interface.program.dump_to_file(filePath, "testprogram.pkl")
        else:
            self.pbd_interface.program.dump_to_file(filePath, str(filename) + '.pkl')
        inputField.clear()
        buttonReply = QMessageBox.question(self, 'PyQt5 message', "Load this program for execution?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if buttonReply == QMessageBox.Yes:
            self.loadNewProgram()

    def updateValidationButtons(self):
        '''
        Enable adding user sync unless the program has not entered the starting state, allow merging of linear motions if
        current and previous primitives are linear motions. Allow converting motions if current primitive is linear or push motion.
        '''
        idx = self.interpreter.next_primitive_index
        if idx == len(self.interpreter.loaded_program.primitives):
            self.validationButtons[0].setEnabled(False)
            self.validationButtons[1].setEnabled(False)
            self.validationButtons[2].setEnabled(False)
            return

        if idx == -1:
            self.validationButtons[0].setEnabled(False)
        else:
            self.validationButtons[0].setEnabled(True)

        curPrimitive = self.interpreter.loaded_program.primitives[idx].__class__.__name__
        if len(self.interpreter.loaded_program.primitives) > 1:
            prevPrimitive = self.interpreter.loaded_program.primitives[idx - 1].__class__.__name__
        else:
            prevPrimitive = None

        if idx > 0:
            if curPrimitive == 'MoveToEE' and prevPrimitive == 'MoveToEE':
                self.validationButtons[1].setEnabled(True)
            else:
                self.validationButtons[1].setEnabled(False)
        else:
            self.validationButtons[1].setEnabled(False)

        
        if idx >= 0 and (curPrimitive == "MoveToEE" or curPrimitive == "MoveToContact"):
            if curPrimitive == "MoveToEE":
                self.motionlabel.setText("Convert the current\nLinear Motion to\na Push Motion")
            else:
                self.motionlabel.setText("Convert the current\nPush Motion to\na Linear Motion")
            self.validationButtons[2].setEnabled(True)
        else:
            self.validationButtons[2].setEnabled(False)

    def add_validation_usersync(self):
        '''
        Add a user sync primitive to the program at the current primitive index of the program interpreter
        '''
        idx = self.interpreter.next_primitive_index
        goal = UserSyncGoal()
        goal.force_threshold = 5.0
        user_sync_primitive = pp.UserSync()
        user_sync_primitive.set_parameter_container(goal)

        arm_index = self.interpreter.loaded_program.primitives[idx].starting_arm_state_index
        gripper_index = self.interpreter.loaded_program.primitives[idx].starting_gripper_state_index
        self.interpreter.loaded_program.primitives.insert(idx, user_sync_primitive)
        self.interpreter.loaded_program.primitives[idx].starting_arm_state_index = arm_index
        self.interpreter.loaded_program.primitives[idx].starting_gripper_state_index = gripper_index

        self.updateValidationButtons()
        self.panda_program_widget.clear()
        for primitive in self.interpreter.loaded_program.primitives:
            self.panda_program_widget.addPrimitiveWidget(primitive, self.interpreter)

        self.panda_program_widget.primitive_widget_list[idx].primitive.status = pp.PandaPrimitiveStatus.READY
        self.panda_program_widget.primitive_widget_list[idx].updateWidget()
        self.panda_program_widget.primitive_widget_list[idx + 1].primitive.status = pp.PandaPrimitiveStatus.NEUTRAL
        self.panda_program_widget.primitive_widget_list[idx + 1].updateWidget()
        self.saveAfterValidation()
        self.updatePandaWidgets()

    def combine_motion_with_previous(self):
        '''
        Merge a linear motion primitive with the current one so that the goal point of the latter motion is the goal point
        of the merged motion
        '''
        idx = self.interpreter.next_primitive_index
        self.interpreter.loaded_program.primitives[idx - 1].parameter_container = self.interpreter.loaded_program.primitives[idx].parameter_container
        self.interpreter.loaded_program.primitives.pop(idx)
        self.interpreter.next_primitive_index = idx - 1
        idx = self.interpreter.next_primitive_index

        self.updateValidationButtons()
        self.panda_program_widget.clear()
        for primitive in self.interpreter.loaded_program.primitives:
            self.panda_program_widget.addPrimitiveWidget(primitive, self.interpreter)
        self.panda_program_widget.primitive_widget_list[idx].primitive.status = pp.PandaPrimitiveStatus.READY
        self.panda_program_widget.primitive_widget_list[idx].updateWidget()
        self.saveAfterValidation()
        self.updatePandaWidgets()

    def convert_motion(self):
        '''
        Convert current linear motion to push motion or vice versa
        '''
        idx = self.interpreter.next_primitive_index
        currentMotion = self.interpreter.loaded_program.primitives[idx]

        if currentMotion.__class__.__name__ == "MoveToEE":
            pose = currentMotion.parameter_container.pose
            goal = MoveToContactGoal()
            goal.pose = pose
            goal.position_speed = self.pbd_interface.default_parameters['move_to_contact_default_position_speed']
            goal.rotation_speed = self.pbd_interface.default_parameters['move_to_contact_default_rotation_speed']
            goal.force_threshold = self.pbd_interface.default_parameters['move_to_contact_default_force_threshold']
            goal.torque_threshold = self.pbd_interface.default_parameters['move_to_contact_default_torque_threshold']
            move_to_contact_primitive = pp.MoveToContact()
            move_to_contact_primitive.set_parameter_container(goal)

            original_arm_index = self.interpreter.loaded_program.primitives[idx].starting_arm_state_index
            original_gripper_index = self.interpreter.loaded_program.primitives[idx].starting_gripper_state_index
            self.interpreter.loaded_program.primitives[idx] = move_to_contact_primitive
            self.interpreter.loaded_program.primitives[idx].starting_arm_state_index = original_arm_index
            self.interpreter.loaded_program.primitives[idx].starting_gripper_state_index = original_gripper_index

        elif currentMotion.__class__.__name__ == "MoveToContact":
            pose = currentMotion.parameter_container.pose
            goal = MoveToEEGoal()
            goal.position_speed = self.pbd_interface.default_parameters['move_to_ee_default_position_speed']
            goal.rotation_speed = self.pbd_interface.default_parameters['move_to_ee_default_rotation_speed']
            move_to_ee_primitive = pp.MoveToEE()
            move_to_ee_primitive.set_parameter_container(goal)

            original_arm_index = self.interpreter.loaded_program.primitives[idx].starting_arm_state_index
            original_gripper_index = self.interpreter.loaded_program.primitives[idx].starting_gripper_state_index
            self.interpreter.loaded_program.primitives[idx] = move_to_ee_primitive
            self.interpreter.loaded_program.primitives[idx].starting_arm_state_index = original_arm_index
            self.interpreter.loaded_program.primitives[idx].starting_gripper_state_index = original_gripper_index

        else:
            print("Incorrect primitive type")
            return

        self.updateValidationButtons()
        self.panda_program_widget.clear()
        for primitive in self.interpreter.loaded_program.primitives:
            self.panda_program_widget.addPrimitiveWidget(primitive, self.interpreter)
        self.panda_program_widget.primitive_widget_list[idx].primitive.status = pp.PandaPrimitiveStatus.READY
        self.panda_program_widget.primitive_widget_list[idx].updateWidget()
        self.saveAfterValidation()
        self.updatePandaWidgets()

    def saveAfterValidation(self):
        '''
        Ask the user if they want to save the program after the modifications done with validation buttons
        '''
        buttonReply = QMessageBox.question(self, 'PyQt5 message', "Would you like to save the modified program?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if buttonReply == QMessageBox.Yes:
            filename, okPressed = QInputDialog.getText(self, "Save Program", "Enter filename for the program:", QLineEdit.Normal, "")
            if okPressed and filename != '':
                path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
                filename = filename + '.pkl'
                self.interpreter.loaded_program.dump_to_file(filepath=path, filename=filename)

    def loadNewProgram(self):
        '''
        If the user wants to load the created program to run program -tab, this function is executed. The previous program widget
        is cleared and updated with primitives of the new program. EUP state machine is reseted to the state STARTUP so we can
        properly start the program with "Go to start state" -button
        '''
        program = deepcopy(self.pbd_interface.program)
        self.interpreter.load_program(program)
        self.interpreter.loaded_program.reset_primitives_history()
        self.state_machine = EUPStateMachine.STARTUP
        self.last_interface_state = None
        self.updatePandaWidgets()
        self.panda_program_widget.clear()
        for primitive in self.interpreter.loaded_program.primitives:
            self.panda_program_widget.addPrimitiveWidget(primitive, self.interpreter)
        self.panda_program_widget.update()

    def deletePreviousPrimitive(self):
        '''
        Deletes previously added primitive and asks the user whether they want to return the robot to the preconditions of the previous primitive.
        '''
        n = len(self.pbd_interface.program.primitives) - 1
        buttonReply = QMessageBox.question(self, 'PyQt5 message', "Return the robot to previous preconditions?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if buttonReply == QMessageBox.Yes:
            self.returnPreviousPreconditions(n)
        self.pbd_interface.program.delete_nth_primitive(n)
        self.program_creation_widget.deleteLastPrimitive()
        self.program_creation_widget.updateWidget()
        self.updatePandaWidgets()

    def returnPreviousPreconditions(self, primitive_index):
        '''
        Assign the respective revert-function for each primitive. This function is executed if the user chose to return the robot
        to previous preconditions after deleting the previous primitive. 
        '''
        revertFunctions = {"MoveToEE": partial(self.pbd_interface.interpreter.revert_move_to_ee, primitive_index),
                           "MoveToContact": partial(self.pbd_interface.interpreter.revert_move_to_contact, primitive_index),
                           "UserSync": partial(self.pbd_interface.interpreter.revert_user_sync, primitive_index),
                           "MoveFingers": partial(self.pbd_interface.interpreter.revert_move_fingers, primitive_index),
                           "ApplyForceFingers": partial(self.pbd_interface.interpreter.revert_apply_force_fingers, primitive_index)
        }
        name = self.pbd_interface.program.primitives[-1].__class__.__name__
        fn = revertFunctions[name]
        fn()

    def addProgramUtilityActions(self):
        '''
        We assign reset- and save-buttons to their respective functions.
        '''
        self.lowerProgramMenu.saveButton.pressed.connect(self.saveProgram)
        self.lowerProgramMenu.resetButton.pressed.connect(partial(self.program_creation_widget.clear, self.pbd_interface))
        self.lowerProgramMenu.resetButton.pressed.connect(self.updatePandaWidgets)

    def checkForInitialization(self):
        '''
        Disable program creation buttons (both the ones that control the robot and add primitives) except for initialization
        if the program is not initialized.
        '''
        self.program_creation_buttons.controlButtons[0].setEnabled(1 - self.pbd_interface.program.initialized)
        for controlButton in self.program_creation_buttons.controlButtons[1:]:
            controlButton.setEnabled(self.pbd_interface.program.initialized)

        for primitiveButton in self.program_creation_buttons.primitiveButtons:
            primitiveButton.setEnabled(self.pbd_interface.program.initialized)

    def checkForRelaxedAndFrozen(self):
        '''
        Depending on the robots state, disable some options to freeze or relax. In addition, prevent the addition
        of new primitives if the robot is not frozen.
        '''
        if self.pbd_interface.relaxed:
            self.program_creation_buttons.controlButtons[2].setEnabled(False)
            for primitiveButton in self.program_creation_buttons.primitiveButtons:
                primitiveButton.setEnabled(False)

        if not self.pbd_interface.relaxed:
            self.program_creation_buttons.controlButtons[1].setEnabled(False)

    def checkEUPState(self):
        '''
        Disable buttons if robot is busy or in error state.
        '''
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
        '''
        Prevent the user from applying multiple Finger Grasps in a row, this causes an error.
        '''
        if len(self.pbd_interface.program.primitives) > 0:
            lastPrimitive = self.pbd_interface.program.primitives[-1]
            if lastPrimitive.__class__.__name__ == "ApplyForceFingers":
                self.program_creation_buttons.primitiveButtons[4].setEnabled(False)

    def disableWithEmptyProgram(self):
        '''
        Disable certain buttons if the program has no primitives.
        '''
        if len(self.pbd_interface.program.primitives) == 0:
            self.lowerProgramMenu.saveButton.setEnabled(False)
            self.lowerProgramMenu.resetButton.setEnabled(False)
            self.program_creation_buttons.deleteButton.setEnabled(False)
        else:
            self.lowerProgramMenu.saveButton.setEnabled(True)
            self.lowerProgramMenu.resetButton.setEnabled(True)
            self.program_creation_buttons.deleteButton.setEnabled(True)

    def announceWorkerDeath(self):
        rospy.logdebug("RIP Worker!")

    def actOnWorkerUpdate(self, progress):
        self.updatePandaWidgets()

    def sizeHint(self):
        return QSize(1280, 720)

    def minimumSizeHint(self):
        return QSize(800, 600)


class TabWidget(QWidget):
    '''
    This widget creates the tabs which split the GUI into various different purposes such as running existing programs
    or creating new ones.
    '''

    def __init__(self, parent):
        super(TabWidget, self).__init__(parent)
        self.layout = QVBoxLayout()
        self.tabWidget = QTabWidget()
        self.runProgramTab = QWidget()
        self.createProgramTab = QWidget()
        self.demonstrationsTab = QWidget()

        self.tabWidget.addTab(self.runProgramTab, "Run Programs")
        self.tabWidget.addTab(self.createProgramTab, "Create Programs")
        self.tabWidget.addTab(self.demonstrationsTab, "Demonstrations")

        self.runProgramTab.layout = QVBoxLayout(self)
        self.runProgramTab.layout.setAlignment(Qt.AlignTop)
        self.createProgramTab.layout = QVBoxLayout(self)
        self.createProgramTab.layout.setAlignment(Qt.AlignTop)
        self.demonstrationsTab.layout = QVBoxLayout(self)
        self.demonstrationsTab.layout.setAlignment(Qt.AlignTop)

        self.runProgramTab.setLayout(self.runProgramTab.layout)
        self.createProgramTab.setLayout(self.createProgramTab.layout)
        self.demonstrationsTab.setLayout(self.demonstrationsTab.layout)

        self.layout.addWidget(self.tabWidget)
        self.setLayout(self.layout)


class ProgramCreationButtons(QWidget):
    '''
    This widget covers buttons for adding new primitives to the program and also for controlling the robot by for example freezing
    or relaxing it.
    '''
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
        '''
        Add buttons for adding new primitives. The used icons are found in resources-folder. 
        '''
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

        # Iterate through list of all used primitives and create button for each
        primitives = [pp.MoveToEE(), pp.MoveToContact(), pp.UserSync(), pp.MoveFingers(), pp.ApplyForceFingers()]
        for primitive in primitives:
            button = QPushButton('', self)
            imagePath = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources', primitive.__class__.__name__ + '.png')
            button.setIcon(QIcon(imagePath))
            button.setIconSize(QSize(80, 80))
            self.primitiveButtonLayout.addWidget(button)
            self.primitiveButtons.append(button)

        # Add button for deleting previous primitive
        self.deleteButton = QPushButton("Delete\nlast primitive")
        self.deleteButton.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed))
        self.deleteButton.setMinimumWidth(150)
        self.deleteButton.setMinimumHeight(80)
        self.deleteButton.setFont(EUPWidget.font)

        self.lowerLayoutWidget = QWidget(self)
        self.lowerLayout = QHBoxLayout(self.lowerLayoutWidget)
        self.lowerLayout.addWidget(self.deleteButton)
        self.lowerLayout.setAlignment(Qt.AlignCenter)
        self.primitiveVerticalLayout.addWidget(self.primitiveButtonRowWidget)
        self.primitiveVerticalLayout.addWidget(self.lowerLayoutWidget)
        self.layout.addWidget(self.primitiveButtonAreaWidget)

    def sizeHint(self):
        return QSize(100, 100)

    def addControlButtons(self):
        '''
        Add buttons for freezing, relaxing the robot.
        '''
        self.controlButtonWidget = QWidget(self)
        self.controlButtons = []
        self.controlButtonLayout = QGridLayout(self.controlButtonWidget)
        self.controlButtonLayout.setAlignment(Qt.AlignRight)
        labels = ["Initialize\nprogram", "Freeze", "Relax", "Relax\nfingers", "Relax\nonly arm", "Relax\nonly wrist"]
        for i in range(len(labels)):
            button = QExpandingPushButton(labels[i], self)
            button.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
            button.setFont(EUPWidget.font)
            self.controlButtonLayout.addWidget(button, i / 3, i % 3)
            self.controlButtons.append(button)
        self.layout.addWidget(self.controlButtonWidget)


class DemonstrationMenu(QWidget):
    '''
    The demonstration menu includes the plots for recording data, buttons for starting the recording, ending the recording,
    emptying plots and entering program creation mode.
    '''

    def __init__(self, parent):
        super(DemonstrationMenu, self).__init__(parent)
        self.datarecorder = Datarecorder(self.parent().pbd_interface)
        self.seg = Segmentation(self.parent().pbd_interface)
        self.initUI()
        self.recording = False
        self.recordingThreadpool = QThreadPool()
        self.parent = self.parent()

    def initUI(self):
        self.layout = QHBoxLayout(self)
        self.layout.setAlignment(Qt.AlignLeft)
        self.buttonAreaWidget = QWidget(self)
        self.buttonArea = QVBoxLayout(self.buttonAreaWidget)
        self.buttonWidget = QWidget(self)
        self.buttonLayout = QHBoxLayout(self.buttonWidget)
        self.addButtonArea()
        self.layout.addWidget(QVerticalLine())
        self.addGraphWidget()
        self.addGraphFunctionality()
        self.addButtonActions()

    def addButtonArea(self):
        '''
        Add buttons of the demonstration menu
        '''
        self.demoButtons = []
        labels = ["Start recording", "Stop recording", "Clear plot", "Program\nCreation", "Relax Fingers", "Return to\nrecording", "Create Program"]

        for label in labels:
            button = QExpandingPushButton(label, self)
            button.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
            button.setFont(EUPWidget.font)
            self.buttonLayout.addWidget(button)
            self.demoButtons.append(button)

        self.buttonArea.addWidget(self.buttonWidget)
        self.dataInputField = QLineEdit()
        self.dataInputField.setPlaceholderText("Enter filename for data to be used for program creation")
        self.dataInputField.setVisible(False)
        self.buttonArea.addWidget(self.dataInputField)
        self.layout.addWidget(self.buttonAreaWidget)

    def saveData(self):
        '''
        Use the input field at the bottom of the GUI to save recorded data as pickle-files
        '''
        inputfield = self.lowerDemoMenu.inputField
        filename = inputfield.text() + '.pkl'
        path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources', 'data')
        self.datarecorder.saveData(path, filename)
        inputfield.clear()
        return filename

    def addButtonActions(self):
        '''
        Set the buttons of demonstration menu either visible or not as well as assigning their respective functions
        '''
        self.demoButtons[4].setVisible(False)
        self.demoButtons[5].setVisible(False)
        self.demoButtons[6].setVisible(False)
        #self.demoButtons[3].setEnabled(False)
        self.demoButtons[0].pressed.connect(self.startRecording)
        self.demoButtons[1].pressed.connect(self.stopRecording)
        self.demoButtons[2].pressed.connect(self.clearPlot)
        self.demoButtons[3].pressed.connect(self.enterProgramCreation)
        self.demoButtons[4].pressed.connect(self.relaxFingers)
        self.demoButtons[5].pressed.connect(self.returnToRecording)
        self.demoButtons[6].pressed.connect(self.createProgram)

    def addGraphWidget(self):
        '''
        This widget handles the plotting of recorded data
        '''
        self.graphwidget = QWidget(self)
        self.graphlayout = QVBoxLayout(self.graphwidget)

        self.velocitygraphwidget = pg.PlotWidget(self)
        self.velocitygraphwidget.setBackground('w')
        self.velocitygraphwidget.setLabel('left', 'v (m/s)', color='black', size=20)
        self.velocitygraphwidget.setLabel('bottom', 'Time (s)', color='black', size=20)

        self.grippergraphwidget = pg.PlotWidget(self)
        self.grippergraphwidget.setBackground('w')
        self.grippergraphwidget.setLabel('left', 'v_g (m/s)', color='black', size=20)
        self.grippergraphwidget.setLabel('bottom', 'Time (s)', color='black', size=20)

        self.layout.addWidget(self.graphwidget)

    def clearPlot(self):
        self.datarecorder.clearPlot([self.velocitygraphwidget, self.grippergraphwidget])
        self.addGraphFunctionality()

    def addGraphFunctionality(self):
        '''
        Assigns the recorded values to variables that should be plotted
        '''
        self.times_ee = self.datarecorder.time_axis_ee
        self.times_gripper = self.datarecorder.time_axis_gripper

        self.velocities = self.datarecorder.ee_velocities
        self.dataLine_v = self.velocitygraphwidget.plot(self.times_ee, self.velocities)
        self.graphlayout.addWidget(self.velocitygraphwidget)

        self.gripperVelocities = self.datarecorder.gripper_velocities
        self.dataLine_g = self.grippergraphwidget.plot(self.times_gripper, self.gripperVelocities)
        self.graphlayout.addWidget(self.grippergraphwidget)

    def relaxFingers(self):
        '''
        Used only if Finger Grasp is executed during demonstrations
        '''
        self.datarecorder.interface.relax_finger()

    def enterProgramCreation(self):
        '''
        Set some buttons hidden when entering program creation
        '''
        self.demoButtons[0].setVisible(False)
        self.demoButtons[1].setVisible(False)
        self.demoButtons[2].setVisible(False)
        self.demoButtons[3].setVisible(False)
        #self.demoButtons[4].setVisible(False)
        self.dataInputField.setVisible(True)
        self.demoButtons[5].setVisible(True)
        self.demoButtons[6].setVisible(True)

    def returnToRecording(self):
        '''
        When returning to recording mode, bring said buttons back to visible
        '''
        self.demoButtons[0].setVisible(True)
        self.demoButtons[1].setVisible(True)
        self.demoButtons[2].setVisible(True)
        self.demoButtons[3].setVisible(True)
        #self.demoButtons[4].setVisible(True)
        self.dataInputField.setVisible(False)
        self.demoButtons[5].setVisible(False)
        self.demoButtons[6].setVisible(False)

    def createProgram(self):
        '''
        Execute the segmentation algorithm to create a program from recorded data
        '''
        self.seg.interface.program.primitives = []
        self.seg.interface.interpreter.program = None
        self.seg.max_deviation = self.parent.regularizationSlider.slider.value()
        self.parent.demo_program_widget.clear()
        filename = self.dataInputField.text()

        # Use recently recorded data if a file of previous data is not given
        if filename == '':
            self.seg.data = {}
            self.seg.data["ee_velocities"] = self.datarecorder.ee_velocities
            self.seg.data["gripper_velocities"] = self.datarecorder.gripper_velocities
            self.seg.data["trajectory_points"] = self.datarecorder.trajectory_points
            self.seg.data["gripper_states"] = self.datarecorder.gripper_states
            self.seg.data["time_axis_ee"] = self.datarecorder.time_axis_ee
            self.seg.data["time_axis_gripper"] = self.datarecorder.time_axis_gripper

        else:
            path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources', 'data')
            self.seg.data = self.seg.loadData(path, filename)

        # Execute segmentation, save program, and load program to interpreter
        self.seg.createSegments()
        resourcepath = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
        self.seg.saveProgram(path=resourcepath, filename="segmentation_test.pkl")
        self.seg.interface.interpreter.load_program(self.seg.interface.program)

        # Update program widget at the top of the GUI
        for primitive in self.seg.interface.program.primitives:
            self.parent.demo_program_widget.addPrimitiveWidget(primitive, self.seg.interface.interpreter)
        self.parent.demo_program_widget.updateWidget()

        # Ask whether the user wants to save the program and load it to run programs tab
        self.AskForSaving()
        buttonReply = QMessageBox.question(self, 'PyQt5 message', "Load this program for execution?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if buttonReply == QMessageBox.Yes:
            self.loadForExecution()

    def AskForSaving(self):
        buttonReply = QMessageBox.question(self, 'PyQt5 message', "Would you like to save the program?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if buttonReply == QMessageBox.Yes:
            filename, okPressed = QInputDialog.getText(self, "Save Program", "Enter filename for the program:", QLineEdit.Normal, "")
            if okPressed and filename != '':
                path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
                filename = filename + '.pkl'
                self.seg.interface.program.dump_to_file(filepath=path, filename=filename)

    def loadForExecution(self):
        '''
        Use the interpreter of run programs tab to load the created program. Update the UI of run programs tab accordingly
        '''
        self.parent.interpreter.load_program(self.seg.interface.program)
        self.parent.interpreter.loaded_program.reset_primitives_history()
        self.parent.state_machine = EUPStateMachine.STARTUP
        self.parent.last_interface_state = None
        self.parent.updatePandaWidgets()
        self.parent.panda_program_widget.clear()
        for primitive in self.seg.interface.program.primitives:
            self.parent.panda_program_widget.addPrimitiveWidget(primitive, self.parent.interpreter)
        self.parent.panda_program_widget.update()

    def startRecording(self):
        self.datarecorder.startRecording(self.dataLine_v, self.dataLine_g)

    def stopRecording(self):
        self.datarecorder.stopRecording()
        self.demoButtons[3].setEnabled(True)


class LowerProgramMenu(QWidget):
    '''
    The lower program menu contain buttons for actions like saving program, reseting program, showing current robot state
    and recovering from error.
    '''

    def __init__(self, parent):
        super(LowerProgramMenu, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.layout = QHBoxLayout(self)
        self.layout.setAlignment(Qt.AlignLeft)
        # Reuse same state widget that was used in run program -tab
        self.stateWidget = PandaStateWidget(self)
        self.layout.addWidget(self.stateWidget)
        self.addControlStateLabels()
        self.layout.addWidget(QVerticalLine())
        self.addProgramUtilities()

    def addControlStateLabels(self):
        '''
        Add labels indicating whether the robot is frozen or relaxed.
        '''
        font = QFont()
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
        '''
        Function which is called after the control state is changed.
        '''
        if not interface.relaxed:
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
        '''
        Add button for saving, reseting and an inputfield so the user can save programs with any name they prefer.
        '''
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
        self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH) * program.get_program_length(),
                                        V_SPACING + PRIMITIVE_HEIGHT)

        # Add primitive Widgets
        for primitive in program.primitives:
            self.addPrimitiveWidget(primitive, self.parent().interpreter)

    def addPrimitiveWidget(self, primitive, interpreter):
        primitive_widget = PandaPrimitiveWidget(self.program_widget, primitive)
        self.primitive_widget_list.append(primitive_widget)
        self.program_widget_layout.addWidget(primitive_widget)

        program_length = interpreter.loaded_program.get_program_length()
        if self.program_widget.width() < (H_SPACING + PRIMITIVE_WIDTH) * program_length:
            self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH) * program_length, V_SPACING +
                                            PRIMITIVE_HEIGHT)

    def updateWidget(self):
        for i, primitive_widget in enumerate(self.primitive_widget_list):
            primitive_widget.updateWidget()
        self.update()

    def clear(self, interface=None):
        '''
        The program widget is emptied. Primitive widgets are deleted by setting their parent to None.
        '''
        for primitive_widget in self.primitive_widget_list:
            primitive_widget.setParent(None)
        self.primitive_widget_list = []
        if interface != None:
            interface.program = pp.PandaProgram("A Panda Program")
            interface.interpreter.loaded_program = interface.program
        self.update()

    def deleteLastPrimitive(self):
        '''
        Function for deleting the last primitive from the program widget.
        '''
        primitiveToDelete = self.primitive_widget_list[-1]
        primitiveToDelete.setParent(None)
        del self.primitive_widget_list[-1]

    def sizeHint(self):
        return QSize((H_SPACING + PRIMITIVE_WIDTH) * MIN_PRIMITIVE, V_SPACING * 3 + PRIMITIVE_HEIGHT)

    def minimumSizeHint(self):
        return QSize((H_SPACING + PRIMITIVE_WIDTH) * MIN_PRIMITIVE, V_SPACING * 3 + PRIMITIVE_HEIGHT)


class PandaStateWidget(QGroupBox):
    # static variables
    sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    status_color = {
        pp.PandaRobotStatus.READY: 'lightseagreen',
        pp.PandaRobotStatus.BUSY: 'gold',
        pp.PandaRobotStatus.ERROR: 'firebrick'
    }

    font = QFont()
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
        pp.PandaPrimitiveStatus.NEUTRAL: '',
        pp.PandaPrimitiveStatus.READY: pp.PandaPrimitiveStatus.READY.name,
        pp.PandaPrimitiveStatus.ERROR: pp.PandaPrimitiveStatus.ERROR.name,
        pp.PandaPrimitiveStatus.EXECUTING: pp.PandaPrimitiveStatus.EXECUTING.name,
        pp.PandaPrimitiveStatus.REVERTING: pp.PandaPrimitiveStatus.REVERTING.name,
        pp.PandaPrimitiveStatus.EXECUTED: ''
    }
    font = QFont()
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
        self.animation.setDuration(2000)  # in ms
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
    font = QFont()
    font.setBold(True)
    font.setPointSize(11)

    readable_parameter_name = {
        'position_speed': 'Motion Speed',
        'force_threshold': 'Collision Threshold',
        'force': 'Grasp Strength',
        'width': 'Finger Distance',
        'regularization': 'Maximum Deviation'
    }

    def __init__(self, parent, name, measure_unit='', available_range=[0.0, 1.0], range_slider_enabled=False, n_ticks=50):
        super(CurrentValueShowingSlider, self).__init__(parent)
        self.measure_unit = measure_unit
        self.available_range = available_range
        self.name = name
        self.n_ticks = n_ticks
        self.range_slider_enabled = range_slider_enabled
        self.initUI()

    def initUI(self):
        self.widget_layout = QGridLayout(self)
        self.slider = FixNumberTicksSlider(self.available_range[0], self.available_range[1], self.n_ticks, Qt.Horizontal)
        if self.range_slider_enabled:
            min_value = self.available_range[0]
            max_value = self.available_range[1]
            step = (max_value - min_value) / self.n_ticks  # TODO: this should be a parameter
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
