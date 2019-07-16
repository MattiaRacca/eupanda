#!/usr/bin/python

import sys
import os
import rospkg
import rospy
import time

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QApplication, QPushButton
from PyQt5.QtCore import Qt

import panda_eup.panda_primitive as pp
from panda_eup.program_interpreter import PandaProgramInterpreter
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


if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('test_gui', anonymous=True)

    interpreter = PandaProgramInterpreter(robot_less_debug=True)
    program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
    interpreter.load_program(pp.load_program_from_file(program_path, 'program.pkl'))

    ex = TestGui(interpreter)
    ex.panda_program_widget.loadPandaInterpreter(interpreter)
    # ex.showMaximized()
    sys.exit(app.exec_())