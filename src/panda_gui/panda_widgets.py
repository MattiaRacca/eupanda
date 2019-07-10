#!/usr/bin/python
from __future__ import division

from panda_eup import panda_primitive as pp

from PyQt5.QtWidgets import QWidget, QLabel, QFrame, QLayout, QGridLayout, QHBoxLayout, QVBoxLayout, QScrollArea, QSizePolicy
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QSize
from PyQt5.QtGui import QColor, QPalette, QPixmap

# Size of Primitive Widget
PRIMITIVE_WIDTH = 100
PRIMITIVE_HEIGHT = 120

# Spacings for the Program Widget
H_SPACING = 20
V_SPACING = 20

# Minimum number of visible primitives on screen
MIN_PRIMITIVE = 5

class PandaProgramWidget(QWidget):
    def __init__(self, parent):
        super(PandaProgramWidget, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.program_size = 0

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
        palette = QPalette()
        palette.setColor(QPalette.Background, QColor("gainsboro"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        self.program_widget.setAutoFillBackground(True)
        self.program_widget.setPalette(palette)

        # Add scrolling area
        self.program_scroll_area.setWidget(self.program_widget)
        self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH)*self.program_size, V_SPACING + PRIMITIVE_HEIGHT)
        # self.program_scroll_area.resize(self.sizeHint())

    def addPrimitive(self):
        primitive_widget = PandaPrimitiveWidget(self)
        self.program_size += 1
        if self.program_widget.width() < (H_SPACING + PRIMITIVE_WIDTH)*self.program_size:
            self.program_widget.setGeometry(0, 0, (H_SPACING + PRIMITIVE_WIDTH)*self.program_size, V_SPACING + PRIMITIVE_HEIGHT)
        self.program_widget_layout.addWidget(primitive_widget)

    def sizeHint(self):
        return QSize((H_SPACING+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, V_SPACING*3 + PRIMITIVE_HEIGHT)

    def minimumSizeHint(self):
        return QSize((H_SPACING+PRIMITIVE_WIDTH)*MIN_PRIMITIVE, V_SPACING*3 + PRIMITIVE_HEIGHT)


class PandaPrimitiveWidget(QFrame):
    def __init__(self, parent):
        super(PandaPrimitiveWidget, self).__init__(parent)
        self.initUI()

    def initUI(self):
        # Create widget subcomponents
        self.primitive_label = QLabel()
        self.status_label = QLabel(self.tr("Status Y"))
        self.primitive_image = QPixmap(80, 80)
        self.primitive_image.fill(Qt.black)
        self.primitive_label.setPixmap(self.primitive_image)

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
        palette = QPalette()
        palette.setColor(QPalette.Background, QColor("ghostwhite"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)

    def sizeHint(self):
        return QSize(PRIMITIVE_WIDTH, PRIMITIVE_HEIGHT)
