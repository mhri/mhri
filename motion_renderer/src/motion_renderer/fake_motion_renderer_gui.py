#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import rospkg
import os
import vtk
import math

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
        self.vtkWidget.GetRenderWindow().SetLineSmoothing(2)
        self.vtkWidget.GetRenderWindow().SetPointSmoothing(2)
        # self.vtkWidget.GetRenderWindow().SetPolygonSmoothing(2)
        self.vtkWidget.GetRenderWindow().AlphaBitPlanesOn()
        self.vtkWidget.GetRenderWindow().SetMultiSamples(32)



        self.eye_ball = []
        self.eye_ball.append(self.add_eye_ball(pose=[-0.8, 0.0, 0.0], orientation=[90, 0, 0]))
        self.eye_ball.append(self.add_eye_ball(pose=[0.8, 0.0, 0.0], orientation=[90, 0, 0]))

        self.eye_lid = []
        self.eye_lid.append(self.add_eye_lid(pose=[-0.8, 0.0, 0.0], orientation=[90, 0, 0]))
        self.eye_lid.append(self.add_eye_lid(pose=[0.8, 0.0, 0.0], orientation=[90, 0, 0]))


        # Initial Pose and Rotation for Eyeball
        self.eye_ball[0].RotateZ(-2.0)
        self.eye_ball[1].RotateZ(2.0)

        self.eye_lid[0][0].RotateX(-30)
        self.eye_lid[0][1].RotateX(30)

        self.eye_lid[1][0].RotateX(-30)
        self.eye_lid[1][1].RotateX(30)


        self.ren.SetBackground(0.1, 0.1, 0.2)
        camera = vtk.vtkCamera();
        camera.SetPosition(0, 0, 10);
        camera.SetFocalPoint(0, 0, 0);

        # self.iren.RemoveAllObservers()
        self.ren.SetActiveCamera(camera);
        self.iren.Initialize()
        self.iren.Start()

    def add_eye_lid(self, pose, orientation):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(os.path.join(rospkg.RosPack().get_path('motion_renderer'), 'resource', 'eyelid.stl'))

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        eyelid_up_actor = vtk.vtkActor()
        eyelid_up_actor.SetMapper(mapper)
        eyelid_up_actor.SetScale(0.0107, 0.0107, 0.0107)
        eyelid_up_actor.GetProperty().SetColor(0.5, 0.5, 0.5)
        eyelid_up_actor.RotateX(-1.0 * orientation[0])
        eyelid_up_actor.RotateY(orientation[1])
        eyelid_up_actor.RotateZ(orientation[2])
        eyelid_up_actor.SetPosition(pose[0], pose[1], pose[2])
        eyelid_up_actor.GetProperty().SetRepresentationToSurface()

        eyelid_down_actor = vtk.vtkActor()
        eyelid_down_actor.SetMapper(mapper)
        eyelid_down_actor.SetScale(0.0107, 0.0107, 0.0107)
        eyelid_down_actor.GetProperty().SetColor(0.5, 0.5, 0.5)
        eyelid_down_actor.RotateX(1.0 * orientation[0])
        eyelid_down_actor.RotateY(orientation[1])
        eyelid_down_actor.RotateZ(orientation[2])
        eyelid_down_actor.SetPosition(pose[0], pose[1], pose[2])
        eyelid_down_actor.GetProperty().SetRepresentationToSurface()

        self.ren.AddActor(eyelid_up_actor)
        self.ren.AddActor(eyelid_down_actor)

        return (eyelid_up_actor, eyelid_down_actor)


    def add_eye_ball(self, pose, orientation):
        # Eye ball
        sphere = vtk.vtkSphereSource()
        sphere.SetThetaResolution(64)
        sphere.SetPhiResolution(64)
        sphere.SetRadius(0.5)

        reader = vtk.vtkJPEGReader()
        reader.SetFileName(os.path.join(rospkg.RosPack().get_path('motion_renderer'), 'resource', 'green_eye.jpg'))

        texture = vtk.vtkTexture()
        texture.SetInputConnection(reader.GetOutputPort())

        map_to_sphere = vtk.vtkTextureMapToSphere()
        map_to_sphere.SetInputConnection(sphere.GetOutputPort())
        map_to_sphere.PreventSeamOn()

        xform = vtk.vtkTransformTextureCoords()
        xform.SetInputConnection(map_to_sphere.GetOutputPort())
        xform.SetScale(1.5, 1.5, 1)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(xform.GetOutputPort())

        # Left Eye Actor
        eye_actor = vtk.vtkActor()
        eye_actor.SetMapper(mapper)
        eye_actor.SetTexture(texture)
        eye_actor.SetPosition(pose[0], pose[1], pose[2])
        eye_actor.RotateX(90.0)
        eye_actor.RotateY(0.0)
        eye_actor.RotateZ(0.0)

        self.ren.AddActor(eye_actor)
        return eye_actor

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
