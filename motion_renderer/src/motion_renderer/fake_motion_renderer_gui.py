#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import rospkg
import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QSurfaceFormat, QOpenGLVersionProfile


class FakeMotionRendererPlugin(Plugin):
    def __init__(self, context):
        super(FakeMotionRendererPlugin, self).__init__(context)
        self.setObjectName('FakeMotionRendererPlugin')

        self._widget = QWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        ui_file = os.path.join(rospkg.RosPack().get_path('motion_renderer'), 'resource', 'fake_motion_renderer.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

        self.color = 0.0

        self._widget.glWidget.paintGL = self.paintGL
        self._widget.glWidget.timerEvent = self.timerEvent
        self._widget.glWidget.startTimer(40)

        self.gl = self._widget.glWidget.context().versionFunctions(QOpenGLVersionProfile((QSurfaceFormat())))
        self.gl.initializeOpenGLFunctions()

    def timerEvent(self, event):
        self._widget.glWidget.update()

    def paintGL(self):

        self.color = self.color + 0.01
        if self.color >= 1.0:
            self.color = 0.0
                    
        self.gl.glClearColor(1.0, self.color, 1.0, 1.0)
        self.gl.glClear(self.gl.GL_COLOR_BUFFER_BIT | self.gl.GL_DEPTH_BUFFER_BIT)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
