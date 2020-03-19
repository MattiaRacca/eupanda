#!/usr/bin/python
import rospy
from qt_gui.plugin import Plugin
from panda_gui.panda_widgets import EUPWidget


class EUPPlugin(Plugin):
    def __init__(self, context):
        super(EUPPlugin, self).__init__(context)
        self._widget = EUPWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.log_loaded_program()
