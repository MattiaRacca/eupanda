#!/usr/bin/python
from __future__ import division

from panda_eup import panda_primitive as pp

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QLayout, QGridLayout, QHBoxLayout, QVBoxLayout, QScrollArea, QSizePolicy
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QSize
from PyQt5.QtGui import QColor, QPalette, QPixmap

PRIMITIVE_WIDTH = 100
PRIMITIVE_HEIGHT = 120
MIN_PRIMITIVE = 10

class PandaProgramWidget(QWidget):
    def __init__(self, parent):
        super(PandaProgramWidget, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.program_size = 0

        self.layout = QHBoxLayout(self)
        self.scroll_area = QScrollArea(self)

        self.layout.addWidget(self.scroll_area)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

        self.program_frame = QWidget(self)
        self.program_frame_layout = QHBoxLayout(self.program_frame)
        self.program_frame_layout.setAlignment(Qt.AlignLeft)

        self.program_frame.setSizePolicy(sizePolicy)

        palette = QPalette()
        palette.setColor(QPalette.Background, QColor("gainsboro"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        
        self.program_frame.setAutoFillBackground(True)
        self.program_frame.setPalette(palette)

        self.scroll_area.setWidget(self.program_frame)
        self.program_frame.setGeometry(0, 0, (20+PRIMITIVE_WIDTH)*self.program_size, 50 + PRIMITIVE_HEIGHT)
        # self.program_frame.resize(self.sizeHint())
        self.scroll_area.resize(self.sizeHint())

    def addPrimitive(self):
        primitive_widget = PandaPrimitiveWidget(self)
        self.program_size += 1
        if self.program_frame.width() < (20+PRIMITIVE_WIDTH)*self.program_size:
            self.program_frame.setGeometry(0, 0, (20+PRIMITIVE_WIDTH)*self.program_size, 50 + PRIMITIVE_HEIGHT)
        self.program_frame_layout.addWidget(primitive_widget)
        print(self.program_frame.sizeHint())

    def sizeHint(self):
        return QSize((20+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, 80 + PRIMITIVE_HEIGHT)

    def minimumSizeHint(self):
        return QSize((20+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, 80 + PRIMITIVE_HEIGHT)


class PandaPrimitiveWidget(QFrame):
    def __init__(self, parent):
        super(PandaPrimitiveWidget, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.primitive_label = QLabel()
        self.status_label = QLabel(self.tr("Status Y"))
        self.primitive_image = QPixmap(80, 80)
        self.primitive_image.fill(Qt.black)
        self.primitive_label.setPixmap(self.primitive_image)

        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(self.primitive_label, 0, 0)
        grid.addWidget(self.status_label, 1, 0)

        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

        self.setLayout(grid)
        self.setFrameShape(QFrame.Panel)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(2)

        palette = QPalette()
        palette.setColor(QPalette.Background, QColor("mintcream"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)

    def sizeHint(self):
        return QSize(PRIMITIVE_WIDTH,PRIMITIVE_HEIGHT)

    def set_state(self, value):
        if value < 50:
            self.status_label.setText("low")
        else:
            self.status_label.setText("high")
