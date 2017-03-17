#!/usr/bin/env python
#-*- encoding: utf8 -*-

import json
import rospy
import actionlib
import random

from mhri_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mhri_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse

class FakeMotionRender:

    def __init__(self):
        rospy.init_node('fake_renderer', anonymous=True)

        try:
            topic_name = rospy.get_param('~topic_name')
        except KeyError as e:
            print('[ERROR] Needed parameter for (topic_name)...')
            quit()

        if 'fake_render_gesture' in rospy.get_name():
            self.GetInstalledGesturesService = rospy.Service(
                "get_installed_gestures",
                GetInstalledGestures,
                self.handle_get_installed_gestures
            )

            self.motion_list = {
                'neutral': ['neutral_motion1'],
                'encourge': ['encourge_motion1'],
                'attention': ['attention_motion1'],
                'consolation': ['consolation_motion1'],
                'greeting': ['greeting_motion1'],
                'waiting': ['waiting_motion1'],
                'advice': ['advice_motion1'],
                'praise': ['praise_motion1'],
                'command': ['command_motion1'],
            }

        self.server = actionlib.SimpleActionServer(
            topic_name, RenderItemAction, self.execute_callback, False)
        self.server.start()

        rospy.loginfo('[%s] initialized...' % rospy.get_name())
        rospy.spin()

    def handle_get_installed_gestures(self, req):
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse(result)


    def execute_callback(self, goal):
        rospy.loginfo('%s rendering requested...' % rospy.get_name())
        result = RenderItemResult()
        feedback = RenderItemFeedback()

        success = True
        loop_count = 0

        print goal

        if 'fake_render_gesture' in rospy.get_name():
            rospy.loginfo('%s rendering gesture [%s]...'%(rospy.get_name(),
                self.motion_list['neutral'][random.randint(0, len(self.motion_list['neutral']) - 1)]))
            loop_count = 30

        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                success = False
                break

            feedback.is_rendering = True
            self.server.publish_feedback(feedback)
            rospy.sleep(0.1)

            loop_count = loop_count - 1
            if loop_count == 0:
                break

        if success:
            result.result = True
            self.server.set_succeeded(result)
            rospy.loginfo('%s rendering completed...' % rospy.get_name())
        else:
            rospy.loginfo('%s rendering canceled...' % rospy.get_name())


if __name__ == '__main__':
    m = FakeMotionRender()
