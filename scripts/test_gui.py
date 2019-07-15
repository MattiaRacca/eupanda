#!/usr/bin/python

import sys
import os
import rospkg
import rospy
import time

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QApplication, QPushButton
from PyQt5.QtCore import Qt

import panda_eup.panda_primitive as pp
from panda_gui.panda_widgets import PandaProgramWidget


class TestGui(QWidget):
    def __init__(self, interpreter):
        super(TestGui, self).__init__()
        self.interpreter = interpreter
        self.initUI()

    def initUI(self):
        self.setWindowTitle('EUP widget')

        self.vbox = QVBoxLayout()
        self.vbox.setAlignment(Qt.AlignTop)
        self.low_buttons = QWidget()
        self.low_buttons_layout = QHBoxLayout()
        self.low_buttons_layout.setAlignment(Qt.AlignCenter)

        self.panda_program_widget = PandaProgramWidget(self)
        self.go_start_button = QPushButton("Go to start pose")
        self.execute_one_step_button = QPushButton("Execute one step")
        self.revert_one_step_button = QPushButton("Revert one step")

        self.go_start_button.clicked.connect(self.go_to_starting_state)
        self.execute_one_step_button.clicked.connect(self.execute_one_step)
        self.revert_one_step_button.clicked.connect(self.revert_one_step)

        self.low_buttons_layout.addWidget(self.go_start_button)
        self.low_buttons_layout.addWidget(self.execute_one_step_button)
        self.low_buttons_layout.addWidget(self.revert_one_step_button)
        self.low_buttons.setLayout(self.low_buttons_layout)

        self.vbox.addWidget(self.panda_program_widget)
        self.vbox.addWidget(self.low_buttons)

        self.setLayout(self.vbox)
        self.show()

    def go_to_starting_state(self):
        self.interpreter.go_to_starting_state()
        self.panda_program_widget.updateWidget()
    def execute_one_step(self):
        self.interpreter.execute_one_step()
        self.panda_program_widget.updateWidget()
    def revert_one_step(self):
        self.interpreter.revert_one_step()
        self.panda_program_widget.updateWidget()


class DebugInterpreter(object):
    def __init__(self):
        super(DebugInterpreter, self).__init__()
        self.loaded_program = None
        self.next_primitive_index = -1

    def load_program(self, program):
        self.loaded_program = program
        self.next_primitive_index = -1
        for primitive in program.primitives:
            primitive.status = pp.PandaPrimitiveStatus.NEUTRAL

    def go_to_starting_state(self):
        if self.loaded_program is None:
            return False
        self.next_primitive_index = 0
        for primitive in self.loaded_program.primitives:
            primitive.status = pp.PandaPrimitiveStatus.NEUTRAL
        return True

    def execute_one_step(self):
        if self.loaded_program is None:
            rospy.logwarn('no program loaded')
            return False

        try:
            primitive_to_execute = self.loaded_program.get_nth_primitive(self.next_primitive_index)
        except pp.PandaProgramException:
            rospy.logwarn('Nothing left to execute OR Empty program OR wrong indexing')
            return False

        primitive_to_execute.status = pp.PandaPrimitiveStatus.EXECUTING
        time.sleep(3)
        result = True

        if result:
            rospy.loginfo('Executed primitive ' + primitive_to_execute.__str__())
            primitive_to_execute.status = pp.PandaPrimitiveStatus.EXECUTED
            self.next_primitive_index += 1
        else:
            rospy.logerr('Error while executing ' + primitive_to_execute.__str__())
            primitive_to_execute.status = pp.PandaPrimitiveStatus.ERROR
            return False
        return True

    def revert_one_step(self):
        if self.loaded_program is None:
            rospy.logwarn('no program loaded')
            return False

        try:
            primitive_to_revert = self.loaded_program.get_nth_primitive(self.next_primitive_index - 1)
        except pp.PandaProgramException:
            rospy.logwarn('Nothing left to revert OR Empty program OR wrong indexing')
            return False

        if not primitive_to_revert.revertible:
            rospy.logwarn('Cannot revert this primitive! the previous primitives was not completely updated...')
            primitive_to_revert.status = pp.PandaPrimitiveStatus.ERROR
            return False

        primitive_to_revert.status = pp.PandaPrimitiveStatus.REVERTING
        time.sleep(3)
        result =True

        if result:
            rospy.loginfo('Reverted primitive ' + primitive_to_revert.__str__())
            primitive_to_revert.status = pp.PandaPrimitiveStatus.NEUTRAL
            self.next_primitive_index -= 1
        else:
            rospy.logerr('Error while reverting ' + primitive_to_revert.__str__())
            primitive_to_revert.status = pp.PandaPrimitiveStatus.ERROR
            return False

        return True


if __name__ == '__main__':
    app = QApplication(sys.argv)

    interpreter = DebugInterpreter()
    program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
    interpreter.load_program(pp.load_program_from_file(program_path, 'program.pkl'))

    ex = TestGui(interpreter)
    ex.panda_program_widget.loadPandaInterpreter(interpreter)
    # ex.showMaximized()
    sys.exit(app.exec_())