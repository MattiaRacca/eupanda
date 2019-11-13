#!/usr/bin/python
from __future__ import division

from enum import Enum
import pickle
import os
from datetime import datetime
import time
from functools import partial

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QHBoxLayout, QVBoxLayout, QSizePolicy, QApplication
from PyQt5.QtCore import Qt, pyqtSignal, QSize
from PyQt5.QtGui import QFont

import rospy

from panda_gui.panda_widgets import EUPWidget, EUPStateMachine, Worker, PandaTuningWidget
from panda_gui.gui_elements import QHorizontalLine, QExpandingPushButton, QVerticalLine
import panda_eup.panda_primitive as pp

import numpy as np
from range_al import range_al as ral


class ALStateMachine(Enum):
    NEUTRAL = 0
    CHOOSING = 1
    QUERY_CHOSEN = 2
    POSING = 3
    UPDATING = 4
    USER_MESSAGE = 5
    LEARNING_ERROR = 6


class ActiveEUPWidget(EUPWidget):
    available_primitives = [pp.ApplyForceFingers, pp.MoveToEE, pp.MoveToContact, pp.MoveFingers, pp.UserSync]
    questionChosen = pyqtSignal(object, str)
    messageSent = pyqtSignal(object, str)
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
            log['learners'] = self.learners

            with open(os.path.join(os.path.expanduser(program_logging_path), filename.format(date)), 'wb') as f:
                pickle.dump(log, f)
            rospy.loginfo('Current program saved in {}'.format(program_logging_path))
        else:
            rospy.logwarn('Could not find rosparam program_logging_path; skipped program logging')

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

        del self.panda_tuning_widget
        del self.interpreter_command_dict['revert_one_step']
        del self.interpreter_command_dict['execute_rest_of_program']
        del self.interpreter_command_dict['revert_to_beginning_of_program']

        for key, page in self.panda_active_tuning_widget.stacks.items():
            if type(page) is PandaActiveTuningPage:
                page.sendAnswer.connect(self.receiveAnswer)

        self.updatePandaWidgets()

    def querySelectionWrapper(self, progress_callback=None):
        self.current_question_count += 1
        try:
            self.current_question = self.learners[self.current_learning_primitive][self.current_learning_parameter].choose_query()
            rospy.loginfo('Learner query is {}'.format(self.current_question))
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
        rospy.loginfo('Learner update: {}'.format(result))
        return result

    def poseWrapper(self):
        self.waitingAnswer.emit(self.interpreter.loaded_program.primitives[self.current_learning_primitive])

    def usermessageWrapper(self, message, progress_callback=None):
        rospy.logdebug('Gonna wait for a while before {}'.format(message))
        current_primitive = self.interpreter.loaded_program.primitives[self.current_learning_primitive]
        self.messageSent.emit(current_primitive, message)
        time.sleep(3)  # TODO: make this a parameter
        return True

    def receiveAnswer(self, answer):
        self.current_answer = answer
        rospy.logdebug('Received the answer... Updating the learner')
        self.execute_learner_command(self.updateWrapper)

    def updateCurrentPrimitive(self):
        self.log_loaded_program(need_to_log=True, partial_log=True)

    def updatePandaWidgets(self):
        rospy.loginfo('{} | {} | {}'.format(self.last_interface_state, self.state_machine, self.learning_state_machine))
        self.programGUIUpdate.emit()
        if self.last_interface_state is not None:
            self.robotStateUpdate.emit(self.last_interface_state)

        ready_primitive = None
        try:
            if self.learning_state_machine != ALStateMachine.CHOOSING and \
                    self.learning_state_machine != ALStateMachine.QUERY_CHOSEN:
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
                self.tuning_timeseries.append(time.time())
                self.questionChosen.emit(current_primitive, current_parameter)
            else:
                self.learning_state_machine = ALStateMachine.LEARNING_ERROR
                rospy.logerr('Learning error - cannot recover from here')
        elif self.learning_state_machine == ALStateMachine.UPDATING:
            if success:
                if self.current_question_count < self.n_questions:
                    message = "Now I'll revert and repeat this primitive to ask a new question"
                else:
                    try:
                        new_param = self.interpreter.loaded_program.primitives[self.current_learning_primitive]. \
                            gui_tunable_parameters[self.current_learning_parameter + 1]
                    except IndexError:
                        if self.current_learning_primitive + 1 >= self.interpreter.loaded_program.get_program_length():
                            message="Now you can go to back to beginning or quit the program"
                        else:
                            message="I'm done tuning this primitive. Let's move to the next primitive!"
                    else:
                        message="Now I'll revert and repeat this primitive to ask questions on a different parameter"
                partial_message = partial(self.usermessageWrapper, message=message)
                partial_message.__name__ = self.usermessageWrapper.__name__
                self.execute_learner_command(partial_message)
            else:
                self.learning_state_machine = ALStateMachine.LEARNING_ERROR
                rospy.logerr('Learning error - cannot recover from here')
        elif self.learning_state_machine == ALStateMachine.USER_MESSAGE and success:
            self.learning_state_machine = ALStateMachine.NEUTRAL
            if self.current_question_count < self.n_questions:
                rospy.logdebug('Reverting to ask new question (n_p: {}/{} - {} - n_q: {}/{})'.
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
                        rospy.logdebug('Moving back to beginning to learn (n_p: {}/{} - {} - n_q: {}/{})'.
                                       format(self.current_learning_primitive + 1,
                                              self.interpreter.loaded_program.get_program_length(),
                                              self.current_learning_parameter,
                                              self.current_question_count,
                                              self.n_questions))
                    else:
                        rospy.logdebug('Moving to the next primitive (n_p: {}/{} - {} - n_q: {}/{})'.
                                       format(self.current_learning_primitive + 1,
                                              self.interpreter.loaded_program.get_program_length(),
                                              self.current_learning_parameter,
                                              self.current_question_count,
                                              self.n_questions))
                        self.execute_learner_command(self.querySelectionWrapper)
                else:
                    rospy.logdebug('Reverting to ask new question: same primitive, different parameter '
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


class PandaActiveTuningWidget(PandaTuningWidget):
    # static variables
    def __init__(self, parent):
        super(PandaActiveTuningWidget, self).__init__(parent=parent,
                                                      range_sliders=False)

    def initUI(self):
        self.stacks = {}
        self.stacks[None] = QWidget(self)
        self.addWidget(self.stacks[None])

        for primitive_type in PandaActiveTuningWidget.tunable_primitives:
            self.stacks[primitive_type] = PandaActiveTuningPage(self, primitive_type)
            self.addWidget(self.stacks[primitive_type])
            self.parent().questionChosen.connect(self.stacks[primitive_type].showQuestion)
            self.parent().messageSent.connect(self.stacks[primitive_type].showMessage)
            self.parent().waitingAnswer.connect(self.stacks[primitive_type].enableAnswering)

        self.setCurrentIndex(0)
        self.setSizePolicy(PandaActiveTuningWidget.sizePolicy)


class PandaActiveTuningPage(QFrame):
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
    font = QFont()
    font.setPointSize(12)
    font.setBold(True)

    not_bold_font = QFont()
    not_bold_font.setPointSize(12)

    message_font = QFont()
    message_font.setPointSize(16)
    message_font.setBold(True)

    def __init__(self, parent, primitive_type):
        super(PandaActiveTuningPage, self).__init__(parent)
        self.primitive_type = primitive_type
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
        title_label.setFont(self.font)
        self.parameter_layout.addWidget(title_label)
        self.parameter_layout.addWidget(QHorizontalLine())
        for param in self.primitive_type.gui_tunable_parameters:
            self.parameter_labels[param] = [QLabel(self.readable_parameter_name[param]), QLabel('unknown')]
            self.parameter_labels[param][0].setFont(self.font)
            self.parameter_labels[param][1].setFont(self.font)
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
            value.setFont(self.font)
            self.answer_buttons_layout.addWidget(value)
            value.clicked.connect(partial(self.communicateAnswer, answer=answers[key]))

        # QUESTION LABEL
        label_size_policy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.value_label = QLabel('current value')
        self.value_label.setWordWrap(True)
        self.value_label.setFont(self.message_font)
        self.value_label.sizeHint = lambda : QSize(200,60)
        self.value_label.setSizePolicy(label_size_policy)
        self.question_label = QLabel('question')
        self.question_label.setWordWrap(True)
        self.question_label.setFont(self.font)
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
            value_statement = 'I will execute this {} now with {} = {:.3f} {}'. \
                format(self.readable_primitive_name[primitive.__class__],
                       self.readable_parameter_name[parameter],
                       self.last_value,
                       primitive.gui_tunable_parameter_units[parameter])
            self.value_label.setText(value_statement)
            self.question_label.setText('')
            self.answer_buttons.setVisible(False)

    def showMessage(self, primitive, message):
        if primitive.__class__ is self.primitive_type:
            self.value_label.setText(message)
            self.value_label.setFont(self.message_font)
            self.question_label.setText('')
            self.answer_buttons.setVisible(False)

    def enableAnswering(self, primitive):
        if primitive.__class__ is self.primitive_type:
            value_statement = '{} executed (with {} = {:.3f} {})'. \
                format(self.readable_primitive_name[primitive.__class__],
                       self.readable_parameter_name[self.last_parameter],
                       self.last_value,
                       primitive.gui_tunable_parameter_units[self.last_parameter])
            self.value_label.setText(value_statement)
            self.value_label.setFont(self.font)
            self.answer_buttons.setVisible(True)

            answer_index = {
                'lower': 0,
                'fine': 1,
                'higher': 2
            }
            for key, value in self.buttons.items():
                value.setText(primitive.gui_tunable_parameter_answer_readable[self.last_parameter][answer_index[key]])
                value.setEnabled(True)
            self.question_label.setFont(self.message_font)
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
                    self.parameter_labels[param][0].setFont(self.font)
                    self.parameter_labels[param][1].setFont(self.font)
                else:
                    self.parameter_labels[param][0].setFont(self.not_bold_font)
                    self.parameter_labels[param][1].setFont(self.not_bold_font)
