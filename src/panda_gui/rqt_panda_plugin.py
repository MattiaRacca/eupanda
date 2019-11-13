#!/usr/bin/python
import rospy
from qt_gui.plugin import Plugin
from panda_gui.panda_widgets import EUPWidget
from panda_gui.active_widgets import ActiveEUPWidget


class EUPPlugin(Plugin):
    def __init__(self, context):
        super(EUPPlugin, self).__init__(context)
        active_eup = rospy.get_param('/active_eup') if rospy.has_param('/active_eup') else False
        self._widget = ActiveEUPWidget() if active_eup else EUPWidget()
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.log_loaded_program()