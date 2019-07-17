#!/usr/bin/python

import sys
import os
import rospkg

from PyQt5.QtWidgets import QApplication
from panda_gui.panda_widgets import EUPWidget


if __name__ == '__main__':
    app = QApplication(sys.argv)

    ex = EUPWidget()
    ex.show()
    # ex.showMaximized()
    sys.exit(app.exec_())