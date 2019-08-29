#!/usr/bin/python
from __future__ import division

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QPushButton, QHBoxLayout, QVBoxLayout, QScrollArea, QSizePolicy,\
    QGroupBox, QApplication,QStackedWidget, QSlider, QGridLayout
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QSize, QThreadPool, pyqtProperty, QPropertyAnimation
from PyQt5.QtGui import QColor, QPalette, QPixmap, QCursor, QFont
from qt_gui.plugin import Plugin

import rospkg
import rospy
from std_msgs.msg import Int32
from franka_control.msg import ErrorRecoveryActionGoal

from panda_eup.program_interpreter import PandaProgramInterpreter
import panda_eup.panda_primitive as pp

import pyttsx3

import os
import traceback
import sys
import pickle
from functools import partial
from enum import Enum
from datetime import datetime
import time

import numpy as np
from range_al import range_al as ral

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
gray_palette.setColor(QPalette.Background, QColor("gainsboro"))
white_palette = QPalette()
white_palette.setColor(QPalette.Background, QColor("ghostwhite"))
executed_primitive_palette = QPalette()
executed_primitive_palette.setColor(QPalette.Background, QColor("cornflowerblue"))
error_palette = QPalette()
error_palette.setColor(QPalette.Background, QColor("firebrick"))


class EUPStateMachine(Enum):
    STARTUP = 0
    STARTUP_BUSY = 1
    OPERATIONAL = 2
    BUSY = 3
    STARTUP_ERROR = 4
    EXECUTION_ERROR = 5
    REVERTING_ERROR = 6


class ALStateMachine(Enum):
    NEUTRAL = 0
    CHOOSING = 1
    QUERY_CHOSEN = 2
    POSING = 3
    UPDATING = 4
    USER_MESSAGE = 5
    LEARNING_ERROR = 6


class EUPPlugin(Plugin):
    def __init__(self, context):
        super(EUPPlugin, self).__init__(context)
        active_eup = rospy.get_param('/active_eup') if rospy.has_param('/active_eup') else False
        self._widget = ActiveEUPWidget() if active_eup else EUPWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.log_loaded_program()


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

    def __init__(self, title='EUP Widget'):
        super(EUPWidget, self).__init__()
        self.setWindowTitle(title)
        self.starting_timestamp = time.time()

        # Creating the interpreter and loading the program
        robotless_debug = rospy.get_param('/robotless_debug') if rospy.has_param('/robotless_debug') else False
        self.interpreter = PandaProgramInterpreter(robotless_debug=robotless_debug)

        if rospy.has_param('/program_path') and rospy.has_param('/program_name'):
            program_path = rospy.get_param('/program_path')
            program_name = rospy.get_param('/program_name')
        else:
            program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
            program_name = 'program.pkl'
            rospy.logwarn('Could not find rosparam program_path OR the program_name; loading the example program')

        self.interpreter.load_program(pp.load_program_from_file(program_path, program_name))

        randomize = False
        if rospy.has_param('/randomize_parameters'):
            randomize = rospy.get_param('/randomize_parameters')

        self.tts_for_primitives = False
        if rospy.has_param('/tts_for_primitives'):
            self.tts_for_primitives = rospy.get_param('/tts_for_primitives')

        # TTS engine
        self.tts_engine = pyttsx3.init()
        voices = self.tts_engine.getProperty('voices')
        self.tts_engine.setProperty('voice', voices[16].id)  # American English
        self.tts_engine.setProperty('volume', 0.8)
        self.tts_engine.setProperty('rate', 180)

        if randomize:
            rospy.loginfo('Going to randomize the primitives parameters... oh dear')
            self.interpreter.loaded_program.randomize_gui_tunable_primitives()

        # Setting up the state machines
        self.state_machine = EUPStateMachine.STARTUP
        self.last_interface_state = None

        # Thread-pool for the interpreter commands
        self.threadpool = QThreadPool()
        rospy.logdebug("Multi-threading with maximum %d threads" % self.threadpool.maxThreadCount())

        self.initUI()

        # Subscriber for the interface status
        self.interface_state_subscriber = rospy.Subscriber("/primitive_interface_node/interface_state", Int32,
                                                           self.interface_state_callback)

    def log_loaded_program(self, need_to_log=True, type_of_primitive=None, name_of_parameter='', partial_log=False):
        if rospy.has_param('/program_logging_path') and need_to_log:
            program_logging_path = rospy.get_param('/program_logging_path')
            date = datetime.fromtimestamp(self.starting_timestamp).strftime('%m%d_%H%M')
            if not os.path.exists(program_logging_path):
                os.makedirs(program_logging_path)
            filename = '{}_partial.pkl' if partial_log else '{}.pkl'
            self.interpreter.loaded_program.dump_to_file(filepath=program_logging_path,
                                                         filename=filename.format(date))
            rospy.loginfo('Current program saved in {}'.format(program_logging_path))
        else:
            rospy.logwarn('Could not find rosparam program_logging_path; skipped program logging')

    def interface_state_callback(self, msg):
        new_interface_status = pp.PandaRobotStatus(msg.data)
        if self.last_interface_state != new_interface_status:
            self.last_interface_state = new_interface_status
            self.updateGUI.emit()

    def initUI(self):
        # Create overall layout
        self.vbox = QVBoxLayout(self)
        self.vbox.setAlignment(Qt.AlignTop)

        # Panda Program Widget on top
        self.panda_program_widget = PandaProgramWidget(self)

        # Parameter tuning frame
        self.panda_tuning_widget = PandaTuningWidget(self)

        # Action button & Robot State Widget at the bottom
        self.low_buttons = QWidget()
        self.low_buttons_layout = QHBoxLayout(self.low_buttons)
        self.low_buttons_layout.setAlignment(Qt.AlignCenter)

        self.robot_state_widget = PandaStateWidget(self)

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

        self.low_buttons_layout.addWidget(self.robot_state_widget)
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['go_to_starting_state'][0])
        self.low_buttons_layout.addWidget(QVerticalLine())
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['execute_one_step'][0])
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['revert_one_step'][0])
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['go_to_current_primitive_preconditions'][0])
        self.low_buttons_layout.addWidget(QVerticalLine())
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['execute_rest_of_program'][0])
        self.low_buttons_layout.addWidget(self.interpreter_command_dict['revert_to_beginning_of_program'][0])

        # Click buttons events handling
        for key, value in self.interpreter_command_dict.items():
            value[0].clicked.connect(value[1])
            value[0].setEnabled(key is 'go_to_starting_state')
            value[0].setVisible(key is not 'go_to_current_primitive_preconditions')
            value[0].setFont(EUPWidget.font)

        # Put everything together
        self.vbox.addWidget(self.panda_program_widget)
        self.vbox.addWidget(self.panda_tuning_widget)
        self.vbox.addWidget(self.low_buttons)

        # Connect update signals
        self.programGUIUpdate.connect(self.panda_program_widget.updateWidget)
        self.robotStateUpdate.connect(self.robot_state_widget.updateWidget)
        self.tuningGUIUpdate.connect(self.panda_tuning_widget.updateWidget)
        self.tuningAccepted.connect(partial(self.log_loaded_program, partial_log=True))
        self.updateGUI.connect(self.updatePandaWidgets)

        for key, page in self.panda_tuning_widget.stacks.items():
            if page.primitive_type is not None:
                page.primitiveTuned.connect(self.updateCurrentPrimitive)

        self.updatePandaWidgets()

    def updateCurrentPrimitive(self, dict):
        if self.state_machine == EUPStateMachine.OPERATIONAL:
            ready_primitive = None
            try:
                ready_primitive = self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index)
                rospy.loginfo('Attempting to tune {}'.format(str(ready_primitive)))
            except pp.PandaProgramException:
                pass

            if ready_primitive is not None:
                for key, value in dict.items():
                    tuned = self.interpreter.loaded_program.update_nth_primitive_parameter(
                        self.interpreter.next_primitive_index, key, value)
                    rospy.logdebug('Tuning parameter {} of a {} primitive: {}'.format(key,\
                                                                                     type(ready_primitive),\
                                                                                     str(tuned)))
                    self.tuningAccepted.emit(tuned, type(ready_primitive), key)
            else:
                rospy.logerr('Are you tuning when you should not?')

    def updatePandaWidgets(self):
        rospy.loginfo('Current EUP state is {}'.format(self.state_machine))
        rospy.loginfo('Current Interface state is {}'.format(self.last_interface_state))

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

    def announceWorkerDeath(self):
        rospy.logdebug("RIP Worker!")

    def actOnWorkerUpdate(self, progress):
        self.updatePandaWidgets()

    def sizeHint(self):
        return QSize(1280, 720)

    def minimumSizeHint(self):
        return QSize(800, 600)


class ActiveEUPWidget(EUPWidget):
    available_primitives = [pp.ApplyForceFingers, pp.MoveToEE, pp.MoveToContact, pp.MoveFingers, pp.UserSync]
    questionChosen = pyqtSignal(object, str)
    waitingAnswer = pyqtSignal(object)

    def __init__(self, title='Active EUP Widget'):
        self.learning_state_machine = ALStateMachine.NEUTRAL
        super(ActiveEUPWidget, self).__init__(title)

        # Active Learners parameter fetching/creation
        self.n_questions = 0  # number of questions for parameter
        if rospy.has_param('/n_questions'):
            self.n_questions = rospy.get_param('/n_questions')
        else:
            raise ValueError('Cannot find rosparam n_questions')

        self.prior_path = None  # path where to find the priors for the learners
        if rospy.has_param('/prior_path'):
            self.prior_path = rospy.get_param('/prior_path')

        self.priors = {}  # priors for the learners
        self.n_buckets = -1  # buckets for the priors (n of bins in the histogram)
        if self.prior_path is not None:
            try:
                with open(self.prior_path, 'rb') as f:
                    self.priors = pickle.load(f)
                for key_primitive, value in self.priors.items():
                    for key_param, prior in value.items():
                        self.n_buckets = prior.shape[0]
                        break
                    break
            except EnvironmentError as e:
                rospy.logerr('Cant find the priors at {}!'.format(self.prior_path))
                raise
        else:
            rospy.logwarn('Priors for parameters not found: will use uninformative priors')
            if rospy.has_param('/n_buckets'):
                self.n_buckets = rospy.get_param('/n_buckets')
            else:
                rospy.logwarn('Cannot find rosparam n_buckets; using 101 buckets default')
                self.n_buckets = 101
            for primitive_type in self.available_primitives:
                self.priors[primitive_type] = {}
                for parameter in primitive_type.gui_tunable_parameters:
                    self.priors[primitive_type][parameter] = np.ones(self.n_buckets)
                    self.priors[primitive_type][parameter] /= np.sum(self.priors[primitive_type][parameter])

        # Create the active learners
        # TODO: the learner type needs to be customizable (name coding?)
        # TODO: all these parameters need to be customizable (from rosparam ideally)
        n_evidence_minmax = 2
        logistic_k_minmax = 50.0
        percentage = 0.03

        self.learners = []
        primitive_counter = 0
        for primitive in self.interpreter.loaded_program.primitives:
            parameter_counter = 0
            self.learners.append([])
            for parameter in primitive.gui_tunable_parameters:
                r = primitive.gui_tunable_parameter_ranges[parameter]
                d = np.linspace(r[0], r[1], self.n_buckets)
                p = self.priors[type(primitive)][parameter]
                self.learners[primitive_counter].append(ral.DivergenceMinMaxLearner(
                    n_evidence_minmax=n_evidence_minmax, logistic_k_minmax=logistic_k_minmax, domain=d,
                    value_distribution=p, profiling=False, safe=True, safe_phi=percentage))
                # TODO: the enforcing of strict range should be customizable
                self.learners[primitive_counter][-1].enforce_strict_range(
                    primitive.gui_tunable_parameter_strict_ranges[parameter])
                parameter_counter += 1
            primitive_counter += 1

        # Active Learning state machine's support variables
        self.current_learning_primitive = 0
        self.current_learning_parameter = 0
        self.current_question_count = 0
        self.current_question = None
        self.current_answer = None

    def initUI(self):
        # create new elements that are updated by updatePandaWidgets
        self.panda_active_tuning_widget = PandaActiveTuningWidget(self)

        # create the standard non-active EUP Widget
        super(ActiveEUPWidget, self).initUI()

        # remove the tuning with sliders and add the active tuning page
        self.vbox.insertWidget(1, self.panda_active_tuning_widget)
        self.vbox.removeWidget(self.panda_tuning_widget)
        self.tuningGUIUpdate.connect(self.panda_active_tuning_widget.updateWidget)

        # remove bunch of stuff that are not needed in the Active Widget
        self.low_buttons_layout.itemAt(6).widget().setParent(None)  # remove a vertical line
        self.interpreter_command_dict['revert_to_beginning_of_program'][0].setParent(None)
        self.interpreter_command_dict['execute_rest_of_program'][0].setParent(None)
        self.interpreter_command_dict['revert_one_step'][0].setParent(None)
        self.panda_tuning_widget.setParent(None)
        del self.interpreter_command_dict['revert_one_step']
        del self.interpreter_command_dict['execute_rest_of_program']
        del self.interpreter_command_dict['revert_to_beginning_of_program']

        for key, page in self.panda_active_tuning_widget.stacks.items():
            if page.primitive_type is not None:
                page.primitiveTuned.connect(self.updateCurrentPrimitive)
                page.sendAnswer.connect(self.receiveAnswer)

        self.updatePandaWidgets()

    def querySelectionWrapper(self, progress_callback=None):
        self.current_question_count += 1
        try:
            self.current_question = self.learners[self.current_learning_primitive][self.current_learning_parameter].choose_query()
            rospy.logdebug('Learner query is {}'.format(self.current_question))
        except:
            return False
        return True

    def updateWrapper(self, progress_callback=None):
        rospy.logdebug('Updating the learner')
        answer = self.current_answer
        try:
            result = self.learners[self.current_learning_primitive][self.current_learning_parameter].update_model(answer)
        except:
            result = False
        rospy.logwarn('Learner update: {}'.format(result))
        return result

    def poseWrapper(self):
        self.waitingAnswer.emit(self.interpreter.loaded_program.primitives[self.current_learning_primitive])

    def usermessageWrapper(self, message, progress_callback=None):
        rospy.logwarn('gonna wait for a while {}'.format(message))
        time.sleep(4)
        rospy.logwarn('ok lets go {}'.format(message))
        return True

    def receiveAnswer(self, answer):
        self.current_answer = answer
        rospy.logdebug('Received the answer... Updating the learner')
        self.execute_learner_command(self.updateWrapper)

    def updatePandaWidgets(self):
        rospy.loginfo('{} | {} | {}'.format(self.last_interface_state, self.state_machine, self.learning_state_machine))
        self.programGUIUpdate.emit()
        if self.last_interface_state is not None:
            self.robotStateUpdate.emit(self.last_interface_state)

        ready_primitive = None
        try:
            if self.learning_state_machine == ALStateMachine.POSING or self.learning_state_machine == ALStateMachine.UPDATING\
                    or self.learning_state_machine == ALStateMachine.NEUTRAL:
                ready_primitive = self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index - 1)
            else:
                ready_primitive = self.interpreter.loaded_program.get_nth_primitive(self.interpreter.next_primitive_index)
        except pp.PandaProgramException:
            pass

        self.tuningGUIUpdate.emit(ready_primitive)
        QApplication.restoreOverrideCursor()

        if self.last_interface_state == pp.PandaRobotStatus.ERROR or \
                self.last_interface_state == pp.PandaRobotStatus.BUSY:
            for key, value in self.interpreter_command_dict.items():
                value[0].setEnabled(False)
            self.panda_active_tuning_widget.setEnabled(False)
        else:
            if self.state_machine == EUPStateMachine.STARTUP:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_starting_state')
                self.panda_active_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.OPERATIONAL:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is not 'go_to_starting_state')
                    value[0].setVisible(key is not 'go_to_current_primitive_preconditions')
                self.panda_active_tuning_widget.setEnabled(True)
                # last primitive executed, disable execute buttons
                if self.interpreter.next_primitive_index == self.interpreter.loaded_program.get_program_length():
                    self.interpreter_command_dict['execute_one_step'][0].setEnabled(False)
                    self.interpreter_command_dict['go_to_starting_state'][0].setEnabled(True)

                if self.learning_state_machine == ALStateMachine.POSING or \
                        self.learning_state_machine == ALStateMachine.USER_MESSAGE:
                    # if you are posing a question or communicating something, don't allow execution
                    for key, value in self.interpreter_command_dict.items():
                        value[0].setEnabled(False)

            elif self.state_machine == EUPStateMachine.STARTUP_BUSY or self.state_machine == EUPStateMachine.BUSY:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(False)
                self.panda_active_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.STARTUP_ERROR:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_starting_state')
                self.panda_active_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.EXECUTION_ERROR:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_current_primitive_preconditions')
                    if key == 'execute_one_step':
                        value[0].setVisible(False)
                    if key == 'go_to_current_primitive_preconditions':
                        value[0].setVisible(True)
                self.panda_active_tuning_widget.setEnabled(False)
            elif self.state_machine == EUPStateMachine.REVERTING_ERROR:
                for key, value in self.interpreter_command_dict.items():
                    value[0].setEnabled(key is 'go_to_current_primitive_preconditions')
                    if key == 'execute_one_step':
                        value[0].setVisible(False)
                    if key == 'go_to_current_primitive_preconditions':
                        value[0].setVisible(True)
                self.panda_active_tuning_widget.setEnabled(False)

    def execute_learner_command(self, command):
        command_keyword = command.__name__
        if self.learning_state_machine == ALStateMachine.NEUTRAL and \
                command_keyword == self.querySelectionWrapper.__name__:
            self.learning_state_machine = ALStateMachine.CHOOSING
            worker = Worker(command)
            worker.signals.result.connect(self.reapLearnerResults)
            worker.signals.finished.connect(self.announceWorkerDeath)
            worker.signals.progress.connect(self.actOnWorkerUpdate)
            self.threadpool.start(worker)
        elif self.learning_state_machine == ALStateMachine.QUERY_CHOSEN and \
                command_keyword == self.poseWrapper.__name__:
            self.learning_state_machine = ALStateMachine.POSING
            command()
        elif self.learning_state_machine == ALStateMachine.POSING and \
                command_keyword == self.updateWrapper.__name__:
            self.learning_state_machine = ALStateMachine.UPDATING
            worker = Worker(command)
            worker.signals.result.connect(self.reapLearnerResults)
            worker.signals.finished.connect(self.announceWorkerDeath)
            worker.signals.progress.connect(self.actOnWorkerUpdate)
            self.threadpool.start(worker)
        elif self.learning_state_machine == ALStateMachine.UPDATING and \
                command_keyword == self.usermessageWrapper.__name__:
            self.learning_state_machine = ALStateMachine.USER_MESSAGE
            worker = Worker(command)
            worker.signals.result.connect(self.reapLearnerResults)
            worker.signals.finished.connect(self.announceWorkerDeath)
            worker.signals.progress.connect(self.actOnWorkerUpdate)
            self.threadpool.start(worker)
        else:
            rospy.logerr('Unknown transition in the learning_state_machine')

    def reapLearnerResults(self, success):
        rospy.logdebug("Learner result: " + str(success))
        if self.learning_state_machine == ALStateMachine.CHOOSING:
            if success:
                self.learning_state_machine = ALStateMachine.QUERY_CHOSEN
                current_primitive = self.interpreter.loaded_program.primitives[self.current_learning_primitive]
                current_parameter = current_primitive.gui_tunable_parameters[self.current_learning_parameter]

                # UPDATE THE VALUE IN THE PRIMITIVE
                current_primitive.update_parameter(current_parameter, self.current_question)
                self.questionChosen.emit(current_primitive, current_parameter)
            else:
                self.learning_state_machine = ALStateMachine.LEARNING_ERROR
                rospy.logerr('Learning error - cannot recover from here')
        elif self.learning_state_machine == ALStateMachine.UPDATING:
            if success:
                if self.current_question_count < self.n_questions:
                    partial_message = partial(self.usermessageWrapper, message='reverting to ask new question, same primitive')
                    partial_message.__name__ = self.usermessageWrapper.__name__
                    self.execute_learner_command(partial_message)
                else:
                    try:
                        new_param = self.interpreter.loaded_program.primitives[self.current_learning_primitive].\
                            gui_tunable_parameters[self.current_learning_parameter + 1]
                    except IndexError:
                        if self.current_learning_primitive + 1 >= self.interpreter.loaded_program.get_program_length():
                            partial_message = partial(self.usermessageWrapper, message='moving back to beginning')
                            partial_message.__name__ = self.usermessageWrapper.__name__
                            self.execute_learner_command(partial_message)
                        else:
                            partial_message = partial(self.usermessageWrapper, message='moving to the next primitive')
                            partial_message.__name__ = self.usermessageWrapper.__name__
                            self.execute_learner_command(partial_message)
                    else:
                        partial_message = partial(self.usermessageWrapper, message='reverting to ask new question, same primitive, different parameter')
                        partial_message.__name__ = self.usermessageWrapper.__name__
                        self.execute_learner_command(partial_message)
            else:
                self.learning_state_machine = ALStateMachine.LEARNING_ERROR
                rospy.logerr('Learning error - cannot recover from here')
        elif self.learning_state_machine == ALStateMachine.USER_MESSAGE and success:
            self.learning_state_machine = ALStateMachine.NEUTRAL
            if self.current_question_count < self.n_questions:
                rospy.loginfo('Reverting to ask new question (n_p: {}/{} - {} - n_q: {}/{})'.
                              format(self.current_learning_primitive + 1,
                                     self.interpreter.loaded_program.get_program_length(),
                                     self.current_learning_parameter,
                                     self.current_question_count,
                                     self.n_questions))
                self.execute_interpreter_command(self.interpreter.revert_one_step)
            else:
                self.current_question_count = 0
                self.current_learning_parameter += 1
                try:
                    new_param = self.interpreter.loaded_program.primitives[self.current_learning_primitive]. \
                        gui_tunable_parameters[self.current_learning_parameter]
                except IndexError:
                    self.current_learning_primitive += 1
                    self.current_learning_parameter = 0
                    if self.current_learning_primitive >= self.interpreter.loaded_program.get_program_length():
                        self.current_learning_primitive = 0
                        self.state_machine = EUPStateMachine.STARTUP
                        rospy.loginfo('Moving back to beginning to learn (n_p: {}/{} - {} - n_q: {}/{})'.
                                      format(self.current_learning_primitive + 1,
                                             self.interpreter.loaded_program.get_program_length(),
                                             self.current_learning_parameter,
                                             self.current_question_count,
                                             self.n_questions))
                    else:
                        rospy.loginfo('Moving to the next primitive (n_p: {}/{} - {} - n_q: {}/{})'.
                                      format(self.current_learning_primitive + 1,
                                             self.interpreter.loaded_program.get_program_length(),
                                             self.current_learning_parameter,
                                             self.current_question_count,
                                             self.n_questions))
                        self.execute_learner_command(self.querySelectionWrapper)
                else:
                    rospy.loginfo('Reverting to ask new question: same primitive, different parameter '
                                  + '(n_p: {}/{} - {} - n_q: {}/{})'.
                                  format(self.current_learning_primitive + 1,
                                         self.interpreter.loaded_program.get_program_length(),
                                         self.current_learning_parameter,
                                         self.current_question_count,
                                         self.n_questions))
                    self.execute_interpreter_command(self.interpreter.revert_one_step)

        self.updatePandaWidgets()

    def reapInterpreterResults(self, success):
        rospy.logdebug("Interpreter result: " + str(success))
        if self.state_machine == EUPStateMachine.STARTUP_BUSY:
            if success:
                self.state_machine = EUPStateMachine.OPERATIONAL
                self.execute_learner_command(self.querySelectionWrapper)
            else:
                self.state_machine = EUPStateMachine.STARTUP_ERROR
        if self.state_machine == EUPStateMachine.BUSY:
            self.state_machine = EUPStateMachine.OPERATIONAL if success else EUPStateMachine.EXECUTION_ERROR
            if self.tts_for_primitives and self.interpreter.last_primitive_attempted is not None:
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
            if success and self.learning_state_machine == ALStateMachine.QUERY_CHOSEN:
                self.execute_learner_command(self.poseWrapper)
            if success and self.learning_state_machine == ALStateMachine.NEUTRAL:
                self.execute_learner_command(self.querySelectionWrapper)

        if self.state_machine == EUPStateMachine.STARTUP_ERROR and success:
            self.state_machine = EUPStateMachine.STARTUP
        if self.state_machine == EUPStateMachine.REVERTING_ERROR and success:
            self.state_machine = EUPStateMachine.OPERATIONAL
            self.execute_interpreter_command(self.interpreter.revert_one_step)
        if self.state_machine == EUPStateMachine.EXECUTION_ERROR and success:
            self.state_machine = EUPStateMachine.OPERATIONAL

        self.updatePandaWidgets()


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
            self.addPrimitiveWidget(primitive)

    def addPrimitiveWidget(self, primitive):
        primitive_widget = PandaPrimitiveWidget(self.program_widget, primitive)
        self.primitive_widget_list.append(primitive_widget)
        self.program_widget_layout.addWidget(primitive_widget)

        program_length = self.parent().interpreter.loaded_program.get_program_length()
        if self.program_widget.width() < (H_SPACING + PRIMITIVE_WIDTH)*program_length:
            self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH)*program_length, V_SPACING +
                                            PRIMITIVE_HEIGHT)

    def updateWidget(self):
        for i, primitive_widget in enumerate(self.primitive_widget_list):
            primitive_widget.updateWidget()
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

    def __init__(self, parent):
        super(PandaTuningWidget, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.stacks = {}
        self.stacks[None] = PandaTuningPage(self, None)
        self.addWidget(self.stacks[None])

        for primitive_type in PandaTuningWidget.tunable_primitives:
            self.stacks[primitive_type] = PandaTuningPage(self, primitive_type)
            self.addWidget(self.stacks[primitive_type])
            self.parent().tuningAccepted.connect(self.stacks[primitive_type].updateAfterTuningAccepted)

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
    primitiveTuned = pyqtSignal(dict)

    def __init__(self, parent, primitive_type):
        super(PandaTuningPage, self).__init__(parent)
        self.primitive_type = primitive_type
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout(self)
        self.sliders = {}
        if self.primitive_type is not None:
            for param in self.primitive_type.gui_tunable_parameters:
                self.sliders[param] = CurrentValueShowingSlider(self, param,
                                                                self.primitive_type.gui_tunable_parameter_units[param],
                                                                self.primitive_type.gui_tunable_parameter_ranges[param])
                self.sliders[param].valueSubmitted.connect(partial(self.signalPrimitiveTuning, param))
                layout.addWidget(self.sliders[param])
        layout.setAlignment(Qt.AlignTop)

    def updatePageFromPritimive(self, primitive):
        if primitive.__class__ is not None:
            for param in primitive.__class__.gui_tunable_parameters:
                self.sliders[param].updateValue(getattr(primitive.parameter_container, param))
                self.sliders[param].slider.setStrictBounds(primitive.gui_tunable_parameter_strict_ranges[param])

    def signalPrimitiveTuning(self, parameter_name, parameter_value):
        self.primitiveTuned.emit({parameter_name: parameter_value})

    def updateAfterTuningAccepted(self, tuned, primitive_type, parameter):
        if self.primitive_type is primitive_type:
            for key, value in self.sliders.items():
                if key == parameter:
                    value.receiveValueConfirmation(tuned)


class PandaActiveTuningWidget(PandaTuningWidget):
    # static variables
    def __init__(self, parent):
        super(PandaActiveTuningWidget, self).__init__(parent)

    def initUI(self):
        self.stacks = {}
        self.stacks[None] = PandaActiveTuningPage(self, None)
        self.addWidget(self.stacks[None])

        for primitive_type in PandaActiveTuningWidget.tunable_primitives:
            self.stacks[primitive_type] = PandaActiveTuningPage(self, primitive_type)
            self.addWidget(self.stacks[primitive_type])
            self.parent().questionChosen.connect(self.stacks[primitive_type].showQuestion)
            self.parent().waitingAnswer.connect(self.stacks[primitive_type].enableAnswering)

        self.setCurrentIndex(0)
        self.setSizePolicy(PandaActiveTuningWidget.sizePolicy)


class PandaActiveTuningPage(QFrame):
    primitiveTuned = pyqtSignal(dict)
    sendAnswer = pyqtSignal(ral.LearnerAnswers)
    readable_parameter_name = {
        'position_speed': 'Motion Speed',
        'force_threshold': 'Collision Threshold',
        'force': 'Grasp Strength',
        'width': 'Finger Distance'
    }
    readable_primitive_name = {
        pp.UserSync: 'User Synchronization',
        pp.MoveToEE: 'Linear Motion',
        pp.MoveFingers: 'Move Fingers',
        pp.MoveToContact: 'Push Motion',
        pp.ApplyForceFingers: 'Finger Grasp'
    }
    not_bold_font = QFont()
    not_bold_font.setPointSize(12)

    def __init__(self, parent, primitive_type):
        super(PandaActiveTuningPage, self).__init__(parent)
        self.primitive_type = primitive_type
        if primitive_type is not None:
            self.initUI()

    def initUI(self):
        self.parameter_widget = QWidget()
        self.dialog_widget = QWidget()
        self.answer_buttons = QWidget()

        self.layout = QHBoxLayout(self)
        self.parameter_layout = QVBoxLayout(self.parameter_widget)
        self.dialog_layout = QVBoxLayout(self.dialog_widget)
        self.answer_buttons_layout = QHBoxLayout(self.answer_buttons)

        self.dialog_layout.setAlignment(Qt.AlignTop)
        self.parameter_layout.setAlignment(Qt.AlignTop)

        # LEFT SIDE OF QWIDGET
        self.parameter_labels = {}
        title_label = QLabel("Current Primitive's \nparameters:")
        title_label.sizeHint = lambda : QSize(200,30)
        title_label.setFont(EUPWidget.font)
        self.parameter_layout.addWidget(title_label)
        self.parameter_layout.addWidget(QHorizontalLine())
        for param in self.primitive_type.gui_tunable_parameters:
            self.parameter_labels[param] = [QLabel(self.readable_parameter_name[param]), QLabel('unknown')]
            self.parameter_labels[param][0].setFont(EUPWidget.font)
            self.parameter_labels[param][1].setFont(EUPWidget.font)
            self.parameter_labels[param][0].sizeHint = lambda : QSize(200,30)
            self.parameter_labels[param][1].sizeHint = lambda : QSize(200,30)
            self.parameter_layout.addWidget(self.parameter_labels[param][0])
            self.parameter_layout.addWidget(self.parameter_labels[param][1])

        # RIGTH SIDE OF QWIDGET
        # ANSWER BUTTONS
        self.buttons = {
            'lower': QExpandingPushButton('lower'),
            'fine': QExpandingPushButton('fine'),
            'higher': QExpandingPushButton('higher')
        }
        answers = {
            'lower': ral.LearnerAnswers.LOWER,
            'fine':ral.LearnerAnswers.FINE,
            'higher': ral.LearnerAnswers.HIGHER
        }
        for key, value in self.buttons.items():
            value.setEnabled(False)
            value.setFont(EUPWidget.font)
            self.answer_buttons_layout.addWidget(value)
            value.clicked.connect(partial(self.communicateAnswer, answer=answers[key]))

        # QUESTION LABEL
        label_size_policy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.value_label = QLabel('current value')
        self.value_label.setWordWrap(True)
        self.value_label.setFont(EUPWidget.font)
        self.value_label.sizeHint = lambda : QSize(200,60)
        self.value_label.setSizePolicy(label_size_policy)
        self.question_label = QLabel('question')
        self.question_label.setWordWrap(True)
        self.question_label.setFont(EUPWidget.font)
        self.question_label.sizeHint = lambda : QSize(200,60)
        self.question_label.setSizePolicy(label_size_policy)
        self.dialog_layout.addWidget(self.value_label)
        self.dialog_layout.addWidget(self.question_label)
        self.dialog_layout.addWidget(self.answer_buttons)

        # putting all together
        self.layout.addWidget(self.parameter_widget)
        self.layout.addWidget(QVerticalLine())
        self.layout.addWidget(self.dialog_widget)

        # support variable
        self.last_value = None
        self.last_parameter = None

    def showQuestion(self, primitive, parameter):
        if primitive.__class__ is self.primitive_type:
            self.last_value = getattr(primitive.parameter_container, parameter)
            self.last_parameter = parameter
            value_statement = 'I will execute this {} now with {} = {:.3f} {}'.\
                format(self.readable_primitive_name[primitive.__class__],
                       self.readable_parameter_name[parameter],
                       self.last_value,
                       primitive.gui_tunable_parameter_units[parameter])
            self.value_label.setText(value_statement)
            self.question_label.setText('')
            self.answer_buttons.setVisible(False)


    def enableAnswering(self, primitive):
        if primitive.__class__ is self.primitive_type:
            value_statement = '{} executed (with {} = {:.3f} {})'.\
                format(self.readable_primitive_name[primitive.__class__],
                       self.readable_parameter_name[self.last_parameter],
                       self.last_value,
                       primitive.gui_tunable_parameter_units[self.last_parameter])
            self.value_label.setText(value_statement)
            self.answer_buttons.setVisible(True)

            answer_index = {
                'lower': 0,
                'fine': 1,
                'higher': 2
            }
            for key, value in self.buttons.items():
                value.setText(primitive.gui_tunable_parameter_answer_readable[self.last_parameter][answer_index[key]])
                value.setEnabled(True)
            self.question_label.setText('How was it?')

    def communicateAnswer(self, answer):
        for key, value in self.buttons.items():
            value.setEnabled(False)
        self.sendAnswer.emit(answer)


    def updatePageFromPritimive(self, primitive):
        if primitive.__class__ is not None:
            for param in primitive.__class__.gui_tunable_parameters:
                s = '{:.3f} {}'.format(getattr(primitive.parameter_container, param),
                                       primitive.gui_tunable_parameter_units[param])
                self.parameter_labels[param][1].setText(s)
                if param == self.last_parameter:
                    self.parameter_labels[param][0].setFont(EUPWidget.font)
                    self.parameter_labels[param][1].setFont(EUPWidget.font)
                else:
                    self.parameter_labels[param][0].setFont(self.not_bold_font)
                    self.parameter_labels[param][1].setFont(self.not_bold_font)


class CurrentValueShowingSlider(QWidget):
    valueSubmitted = pyqtSignal(float)
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

    def __init__(self, parent, name, measure_unit='', available_range=[0.0, 1.0]):
        super(CurrentValueShowingSlider, self).__init__(parent)
        self.measure_unit = measure_unit
        self.available_range = available_range
        self.name = name
        self.initUI()

    def initUI(self):
        self.widget_layout = QGridLayout(self)

        self.slider = FixNumberTicksSlider(self.available_range[0], self.available_range[1], 50, Qt.Horizontal)

        self.current_value_label = QLabel('???')
        self.stored_value_label = QLabel('???')
        self._current_label = QLabel('Current\n Value')
        self._stored_label = QLabel('Stored\n Value')
        self.name_label = QLabel(CurrentValueShowingSlider.readable_parameter_name[self.name])

        self.current_value_label.setFont(CurrentValueShowingSlider.font)
        self.stored_value_label.setFont(CurrentValueShowingSlider.font)
        self._current_label.setFont(CurrentValueShowingSlider.font)
        self._stored_label.setFont(CurrentValueShowingSlider.font)
        self.name_label.setFont(CurrentValueShowingSlider.font)

        # TODO: Better naming of parameters
        # TODO: more visible labels (same style of other elements)
        
        self.current_value_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)
        self.stored_value_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)
        self._current_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)
        self._stored_label.setFixedWidth(CurrentValueShowingSlider.LABEL_WIDTH)

        self.current_value_label.setAlignment(Qt.AlignCenter)
        self.stored_value_label.setAlignment(Qt.AlignCenter)
        self._current_label.setAlignment(Qt.AlignCenter)
        self._stored_label.setAlignment(Qt.AlignCenter)

        self.submit_parameter_value = QPushButton('Submit\n new value')
        self.submit_parameter_value.setSizePolicy(QSizePolicy(QSizePolicy.Fixed, QSizePolicy.MinimumExpanding))
        self.submit_parameter_value.setFont(CurrentValueShowingSlider.font)

        self.widget_layout.addWidget(self.name_label, 1, 1)
        self.widget_layout.addWidget(self.slider, 2, 1)
        self.widget_layout.addWidget(self.current_value_label, 2, 2)
        self.widget_layout.addWidget(self._current_label, 1, 2)
        self.widget_layout.addWidget(self.stored_value_label, 2, 3)
        self.widget_layout.addWidget(self._stored_label, 1, 3)
        self.widget_layout.addWidget(self.submit_parameter_value, 1, 4, 2, 4)

        self.slider.doubleValueChanged.connect(self.updateLabel)
        self.submit_parameter_value.clicked.connect(self.submitValue)

    def updateValue(self, value):
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

    def submitValue(self):
        value = self.slider.value()
        self.valueSubmitted.emit(value)

    def receiveValueConfirmation(self, tuned):
        if tuned:
            value = self.slider.value()
            self.stored_value_label.setText('{:.3f} {}'.format(value, self.measure_unit))
            self.updateLabel(value)
        self.update()


class QVerticalLine(QFrame):
    def __init__(self, parent=None):
        super(QVerticalLine, self).__init__(parent)
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)

class QHorizontalLine(QFrame):
    def __init__(self, parent=None):
        super(QHorizontalLine, self).__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class QExpandingPushButton(QPushButton):
    def __init__(self, text='', parent=None):
        super(QExpandingPushButton, self).__init__(text, parent)
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

    def sizeHint(self):
        return QSize(10, PRIMITIVE_HEIGHT)


class FixNumberTicksSlider(QSlider):

    # create our our signal that we can connect to if necessary
    doubleValueChanged = pyqtSignal(float)

    def __init__(self, lowerbound, upperbound, number_of_ticks=100, *args, **kargs):
        super(FixNumberTicksSlider, self).__init__( *args, **kargs)
        self.setMinimum(0)
        self.setMaximum(number_of_ticks)

        self._lowerbound = lowerbound
        self._upperbound = upperbound
        self._number_of_ticks = number_of_ticks
        self._range = upperbound - lowerbound
        self._real_step = self._range/self._number_of_ticks
        self.setStrictBounds([None, None])

        self.valueChanged.connect(self.emitDoubleValueChanged)
        self.valueChanged.connect(self.restrictMove)
        self.setTickPosition(QSlider.TicksBothSides)

    def setStrictBounds(self, strict_bounds):
        strict_lowerbound = strict_bounds[0]
        strict_upperbound = strict_bounds[1]
        if (strict_lowerbound is None) or (strict_lowerbound <= self._lowerbound):
            self._strict_lowerbound = self._lowerbound
        elif strict_lowerbound > self._upperbound:
            raise ValueError
        else:
            self._strict_lowerbound = strict_lowerbound

        if (strict_upperbound is None) or (strict_upperbound >= self._upperbound):
            self._strict_upperbound = self._upperbound
        elif strict_upperbound < self._lowerbound:
            raise ValueError
        else:
            self._strict_upperbound = strict_upperbound

    def emitDoubleValueChanged(self):
        value = self._lowerbound + float(super(FixNumberTicksSlider, self).value())*self._real_step
        self.doubleValueChanged.emit(value)

    def value(self):
        return self._lowerbound + float(super(FixNumberTicksSlider, self).value())*self._real_step

    def setValue(self, value):
        super(FixNumberTicksSlider, self).setValue(int((value - self._lowerbound)/self._real_step))

    def restrictMove(self):
        value = self._lowerbound + float(super(FixNumberTicksSlider, self).value())*self._real_step
        if not(self._strict_lowerbound <= value <= self._strict_upperbound):
            if value >= self._strict_upperbound:
                value = self._strict_upperbound
            if value <= self._strict_lowerbound:
                value = self._strict_lowerbound
            super(FixNumberTicksSlider, self).setSliderPosition(int((value - self._lowerbound)/self._real_step))


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
