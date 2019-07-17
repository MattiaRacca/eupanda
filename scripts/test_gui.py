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
from panda_gui.panda_widgets import EUPWidget


if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('test_gui', anonymous=True)

    interpreter = PandaProgramInterpreter(robot_less_debug=True)
    program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
    interpreter.load_program(pp.load_program_from_file(program_path, 'program.pkl'))

    ex = EUPWidget(interpreter)
    ex.show()
    # ex.showMaximized()
    sys.exit(app.exec_())