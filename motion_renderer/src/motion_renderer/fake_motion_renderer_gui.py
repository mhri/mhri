#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import rospkg
import os
import vtk

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFrame
from .QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


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

        self.frame = QFrame()
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        self._widget.verticalLayout.addWidget(self.vtkWidget)

        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        source = vtk.vtkSphereSource()
        source.SetCenter(0, 0, 0)
        source.SetRadius(0.1)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        self.ren.AddActor(actor)

        self.ren.ResetCamera()
        self.iren.Initialize()


    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
