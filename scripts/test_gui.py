#!/usr/bin/python
from panda_gui import panda_widgets as pw
import sys
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSlider, QApplication, QPushButton
from PyQt5.QtCore import Qt

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

        self.panda_program = pw.PandaProgramWidget(self)
        self.button.clicked.connect(self.panda_program.addPrimitive)

        self.vbox.addWidget(self.panda_program)
        self.vbox.addWidget(self.sld)
        self.vbox.addWidget(self.button)
        self.vbox.addWidget(pw.PandaPrimitiveWidget(self))

        self.setLayout(self.vbox)
        self.show()

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = TestGui()
    ex.showMaximized()
    sys.exit(app.exec_())