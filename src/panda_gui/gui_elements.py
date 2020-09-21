#!/usr/bin/python
from __future__ import division

from PyQt5.QtWidgets import QWidget, QFrame, QPushButton, QVBoxLayout, QSizePolicy, QSlider
from PyQt5.QtCore import pyqtSignal, QSize
import qt_range_slider.qtRangeSlider as qtRangeSlider

PRIMITIVE_HEIGHT = 120


class RangeSliderTestWidget(QWidget):
    def __init__(self, title='RangeSliderTestWidget'):
        super(RangeSliderTestWidget, self).__init__()
        self.setWindowTitle(title)
        self.layout = QVBoxLayout(self)
        self.range_slider = qtRangeSlider.QHRangeSlider(slider_range=[0, 100, 1], values=[20, 80],
                                                        strict_range=[10, 80], parent=self)
        self.layout.addWidget(self.range_slider)
        self.range_slider.rangeChanged.connect(self.report)

    def report(self, min, max):
        print('slider moved from {} {}'.format(min, max))

    def log_loaded_program(self):
        pass


class QVerticalLine(QFrame):
    def __init__(self, parent=None):
        super(QVerticalLine, self).__init__(parent)
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)


class QHorizontalLine(QFrame):
    def __init__(self, parent=None):
        super(QHorizontalLine, self).__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class QExpandingPushButton(QPushButton):
    def __init__(self, text='', parent=None):
        super(QExpandingPushButton, self).__init__(text, parent)
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Fixed)
        self.setSizePolicy(sizePolicy)

    def sizeHint(self):
        return QSize(10, PRIMITIVE_HEIGHT)


class FixNumberTicksSlider(QSlider):

    # create our our signal that we can connect to if necessary
    doubleValueChanged = pyqtSignal(float)

    def __init__(self, lowerbound, upperbound, number_of_ticks=100, *args, **kargs):
        super(FixNumberTicksSlider, self).__init__(*args, **kargs)
        self.setMinimum(0)
        self.setMaximum(number_of_ticks)

        self._lowerbound = lowerbound
        self._upperbound = upperbound
        self._number_of_ticks = number_of_ticks
        self._range = upperbound - lowerbound
        self._real_step = self._range / self._number_of_ticks
        self.setStrictBounds([None, None])

        self.valueChanged.connect(self.emitDoubleValueChanged)
        self.valueChanged.connect(self.restrictMove)
        self.setTickPosition(QSlider.TicksBothSides)

    def setStrictBounds(self, strict_bounds):
        strict_lowerbound = strict_bounds[0]
        strict_upperbound = strict_bounds[1]
        if (strict_lowerbound is None) or (strict_lowerbound <= self._lowerbound):
            self._strict_lowerbound = self._lowerbound
        elif strict_lowerbound > self._upperbound:
            raise ValueError
        else:
            self._strict_lowerbound = strict_lowerbound

        if (strict_upperbound is None) or (strict_upperbound >= self._upperbound):
            self._strict_upperbound = self._upperbound
        elif strict_upperbound < self._lowerbound:
            raise ValueError
        else:
            self._strict_upperbound = strict_upperbound

    def emitDoubleValueChanged(self):
        value = self._lowerbound + float(super(FixNumberTicksSlider, self).value()) * self._real_step
        self.doubleValueChanged.emit(value)

    def value(self):
        return self._lowerbound + float(super(FixNumberTicksSlider, self).value()) * self._real_step

    def setValue(self, value):
        super(FixNumberTicksSlider, self).setValue(int((value - self._lowerbound) / self._real_step))

    def restrictMove(self):
        value = self._lowerbound + float(super(FixNumberTicksSlider, self).value()) * self._real_step
        if not(self._strict_lowerbound <= value <= self._strict_upperbound):
            if value >= self._strict_upperbound:
                value = self._strict_upperbound
            if value <= self._strict_lowerbound:
                value = self._strict_lowerbound
            where = int((value - self._lowerbound) / self._real_step)
            super(FixNumberTicksSlider, self).setSliderPosition(where)
            value = self._lowerbound + float(super(FixNumberTicksSlider, self).value()) * self._real_step
            # TODO: HORRIBLE HACK BECAUSE OF ROUNDINGS. But I don't want to loose time on this
            if not(self._strict_lowerbound <= value <= self._strict_upperbound):
                super(FixNumberTicksSlider, self).setSliderPosition(where + 1)
