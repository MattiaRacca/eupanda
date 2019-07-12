#!/usr/bin/python

import sys
import os
import rospkg

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSlider, QApplication, QPushButton
from PyQt5.QtCore import Qt

import panda_eup.panda_primitive as eup_pp
from panda_gui.panda_widgets import PandaProgramWidget


class TestGui(QWidget):
    def __init__(self):
        super(TestGui, self).__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('EUP widget')

        self.vbox = QVBoxLayout()
        self.vbox.setAlignment(Qt.AlignTop)

        self.button = QPushButton("add primitive")
        self.sld = QSlider(Qt.Horizontal)
        self.sld.setFocusPolicy(Qt.NoFocus)
        self.sld.setRange(1, 100)
        self.sld.setMaximumSize(100,120)
        self.sld.setValue(50)

        self.panda_program_widget = PandaProgramWidget(self)
        # self.button.clicked.connect(self.panda_program_widget.loadPandaProgram(current_program))

        self.vbox.addWidget(self.panda_program_widget)
        self.vbox.addWidget(self.sld)
        self.vbox.addWidget(self.button)

        self.setLayout(self.vbox)
        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = TestGui()
    program_path = os.path.join(rospkg.RosPack().get_path('panda_pbd'), 'resources')
    ex.panda_program_widget.loadPandaProgram(eup_pp.load_program_from_file(program_path, 'program.pkl'))
    # ex.showMaximized()
    sys.exit(app.exec_())