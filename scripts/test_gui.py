#!/usr/bin/python
from panda_gui import panda_widgets as pw
import sys
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSlider, QApplication
from PyQt5.QtCore import Qt

class TestGui(QWidget):
    def __init__(self):
        super(TestGui, self).__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 200, 120)
        self.setWindowTitle('Primitive widget')

        vbox = QVBoxLayout()
        vbox.addStretch(1)

        sld = QSlider(Qt.Horizontal)
        sld.setFocusPolicy(Qt.NoFocus)
        sld.setRange(1, 100)
        sld.setMaximumSize(100,120)
        sld.setValue(50)

        self.wid = pw.PandaPrimitiveWidget("Very long text Very long text")
        sld.valueChanged[int].connect(self.wid.set_state)

        vbox.addWidget(self.wid)
        vbox.addWidget(sld)

        self.setLayout(vbox)
        self.show()

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = TestGui()
    sys.exit(app.exec_())