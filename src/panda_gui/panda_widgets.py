#!/usr/bin/python
from __future__ import division

from panda_eup import panda_primitive as pp

from PyQt5.QtWidgets import QLabel, QFrame, QGridLayout
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtGui import QColor, QPalette, QPixmap


class PandaProgramWidget(QFrame):
    def __init__(self):
        super(PandaProgramWidget, self).__init__()


class PandaPrimitiveWidget(QFrame):
    def __init__(self, primitive_type):
        super(PandaPrimitiveWidget, self).__init__()
        self.initUI(primitive_type)

    def initUI(self, primitive_type):
        self.primitive_label = QLabel()
        self.status_label = QLabel(self.tr("Status Y"))
        self.primitive_image = QPixmap(80, 80)
        self.primitive_image.fill(Qt.black)
        self.primitive_label.setPixmap(self.primitive_image)

        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(self.primitive_label, 0, 0)
        grid.addWidget(self.status_label, 1, 0)

        self.setLayout(grid)
        self.setFrameShape(QFrame.Panel)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(5)
        self.setGeometry(0, 0, 100, 120)
        self.setMinimumSize(100, 120)
        self.setMaximumSize(100, 120)

        palette = QPalette()
        palette.setColor(QPalette.Background, QColor("mintcream"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)

    def set_state(self, value):
        if value < 50:
            self.status_label.setText("low")
        else:
            self.status_label.setText("high")
