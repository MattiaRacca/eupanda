#!/usr/bin/python
from __future__ import division

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QPushButton, QHBoxLayout, QVBoxLayout, QScrollArea, QSizePolicy,\
    QGroupBox
from PyQt5.QtCore import Qt, QObject, QRunnable, pyqtSignal, pyqtSlot, QSize, QThreadPool
from PyQt5.QtGui import QColor, QPalette, QPixmap
from qt_gui.plugin import Plugin

from panda_eup.program_interpreter import PandaProgramInterpreter
import panda_eup.panda_primitive as pp

import os
import rospkg
import rospy
import threading
import traceback
import sys

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
current_index_palette = QPalette()
current_index_palette.setColor(QPalette.Background, QColor("lightskyblue"))
executed_primitive_palette = QPalette()
executed_primitive_palette.setColor(QPalette.Background, QColor("cornflowerblue"))


class EUPPlugin(Plugin):
    def __init__(self, context):
        super(EUPPlugin, self).__init__(context)
        self._widget = EUPWidget()

        # Add widget to the user interface
        context.add_widget(self._widget)


class EUPWidget(QWidget):
    programUpdate = pyqtSignal(int)
    goToStartStateSignal = pyqtSignal()
    executeOneStepSignal = pyqtSignal()
    revertOneStepSignal = pyqtSignal()

    def __init__(self, title='EUP Widget'):
        super(EUPWidget, self).__init__()
        self.setWindowTitle(title)

        self.interpreter = PandaProgramInterpreter(robot_less_debug=True)
        program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
        self.interpreter.load_program(pp.load_program_from_file(program_path, 'program.pkl'))

        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

        self.initUI()

    def initUI(self):
        # Create overall layout
        self.vbox = QVBoxLayout(self)
        self.vbox.setAlignment(Qt.AlignTop)

        # Panda Widget on top
        self.panda_program_widget = PandaProgramWidget(self)

        # Parameter tuning frame in the middle
        self.adjust_parameters_frame = QGroupBox(self)
        self.adjust_parameters_frame.setTitle('Primitive parameters:')
        size_policy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.adjust_parameters_frame.setSizePolicy(size_policy)

        # Action button at the bottom
        self.low_buttons = QWidget()
        self.low_buttons_layout = QHBoxLayout()
        self.low_buttons_layout.setAlignment(Qt.AlignCenter)
        self.go_start_button = QExpandingPushButton("Go to start pose")
        self.execute_one_step_button = QExpandingPushButton("Execute one step")
        self.revert_one_step_button = QExpandingPushButton("Revert one step")
        self.execute_rest_button = QExpandingPushButton("Execute rest of program")
        self.revert_to_beginning_button = QExpandingPushButton("Revert to beginning")
        self.low_buttons_layout.addWidget(self.go_start_button)
        self.low_buttons_layout.addWidget(QVerticalLine())
        self.low_buttons_layout.addWidget(self.execute_one_step_button)
        self.low_buttons_layout.addWidget(self.revert_one_step_button)
        self.low_buttons_layout.addWidget(QVerticalLine())
        self.low_buttons_layout.addWidget(self.execute_rest_button)
        self.low_buttons_layout.addWidget(self.revert_to_beginning_button)
        self.low_buttons.setLayout(self.low_buttons_layout)

        # Click buttons events handling
        self.go_start_button.clicked.connect(self.go_to_starting_state)
        self.execute_one_step_button.clicked.connect(self.execute_one_step)
        self.revert_one_step_button.clicked.connect(self.revert_one_step)

        # Put everything together
        self.vbox.addWidget(self.panda_program_widget)
        self.vbox.addWidget(self.adjust_parameters_frame)
        self.vbox.addWidget(self.low_buttons)

        # Connect update signals
        self.programUpdate.connect(self.panda_program_widget.updateWidget)
        self.programUpdate.emit(self.interpreter.next_primitive_index)

    def go_to_starting_state(self):
        worker = Worker(self.interpreter.go_to_starting_state) # Any other args, kwargs are passed to the run function
        worker.signals.result.connect(self.reapWorkerResults)
        worker.signals.finished.connect(self.announceWorkerDeath)
        # worker.signals.progress.connect(self.progress_fn)

        self.threadpool.start(worker)
        self.programUpdate.emit(self.interpreter.next_primitive_index)
    def execute_one_step(self):
        worker = Worker(self.interpreter.execute_one_step)
        worker.signals.result.connect(self.reapWorkerResults)
        worker.signals.finished.connect(self.announceWorkerDeath)

        self.threadpool.start(worker)
        self.programUpdate.emit(self.interpreter.next_primitive_index)
    def revert_one_step(self):
        worker = Worker(self.interpreter.revert_one_step)
        worker.signals.result.connect(self.reapWorkerResults)
        worker.signals.finished.connect(self.announceWorkerDeath)

        self.threadpool.start(worker)
        self.programUpdate.emit(self.interpreter.next_primitive_index)

    def reapWorkerResults(self, result):
        rospy.loginfo("WORKER result: " + str(result))

    def announceWorkerDeath(self):
        rospy.loginfo("RIP WORKER!")
        self.programUpdate.emit(self.interpreter.next_primitive_index)

    def sizeHint(self):
        return QSize(1280, 720)

    def minimumSizeHint(self):
        return QSize(800, 600)


class PandaProgramWidget(QGroupBox):
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
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

        # Create container and layout for PandaPrimitiveWidgets
        self.program_widget = QWidget(self)
        self.program_widget_layout = QHBoxLayout(self.program_widget)
        self.program_widget_layout.setAlignment(Qt.AlignLeft)
        self.program_widget.setSizePolicy(sizePolicy)

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

    def updateWidget(self, index):
        for i, primitive_widget in enumerate(self.primitive_widget_list):
            primitive_widget.updateWidget(i, index)
        self.update()

    def sizeHint(self):
        return QSize((H_SPACING+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, V_SPACING*3 + PRIMITIVE_HEIGHT)

    def minimumSizeHint(self):
        return QSize((H_SPACING+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, V_SPACING*3 + PRIMITIVE_HEIGHT)


class PandaPrimitiveWidget(QFrame):
    def __init__(self, parent, panda_primitive):
        super(PandaPrimitiveWidget, self).__init__(parent)
        self.initUI(panda_primitive)

    def initUI(self, panda_primitive):
        # Create widget subcomponents
        self.primitive = panda_primitive
        self.primitive_label = QLabel()
        self.status_label = QLabel(str(panda_primitive.status.name))

        # Fetch fitting icon for primitive
        primitive_icon_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources',
                                           panda_primitive.__class__.__name__ + '.png')

        primitive_image = QPixmap(primitive_icon_path)
        self.primitive_label.setPixmap(primitive_image)

        # Add vertical layout
        layout = QVBoxLayout(self)
        layout.addWidget(self.primitive_label)
        layout.addWidget(self.status_label)
        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

        # Beautify QFrame and Color
        self.setFrameShape(QFrame.Panel)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(2)

        self.setAutoFillBackground(True)
        self.setPalette(white_palette)

    def sizeHint(self):
        return QSize(PRIMITIVE_WIDTH, PRIMITIVE_HEIGHT)

    def updateWidget(self, own_index, program_current_index):
        self.status_label.setText(str(self.primitive.status.name))
        if  own_index < program_current_index:
            self.setPalette(executed_primitive_palette)
        elif own_index > program_current_index:
            self.setPalette(gray_palette)
        else:
            self.setPalette(current_index_palette)
        self.update()

class QVerticalLine(QFrame):
    def __init__(self, parent=None):
        super(QVerticalLine, self).__init__(parent)
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)

class QExpandingPushButton(QPushButton):
    def __init__(self, text='', parent=None):
        super(QExpandingPushButton, self).__init__(text, parent)
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

    def sizeHint(self):
        return QSize(10, PRIMITIVE_HEIGHT/2)


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
        # self.kwargs['progress_callback'] = self.signals.progress

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

