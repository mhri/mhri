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
            topic_name = rospy.get_param('~action_name')
        except KeyError as e:
            print('[ERROR] Needed parameter for (topic_name)...')
            quit()

        if 'render_gesture' in rospy.get_name():
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
        rospy.loginfo('\033[95m%s\033[0m rendering requested...' % rospy.get_name())
        result = RenderItemResult()
        feedback = RenderItemFeedback()

        success = True
        loop_count = 0

        if 'render_gesture' in rospy.get_name():
            (cmd, item_name) = goal.data.split(':')
            if cmd == 'tag':
                rospy.loginfo('\033[94m[%s]\033[0m rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                    cmd,
                    self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]))
            elif cmd == 'play':
                find_result = False
                for k, v in self.motion_list.items():
                    if item_name in v:
                        find_result = True

                if find_result:
                    rospy.loginfo('\033[94m[%s]\033[0m rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                        cmd, item_name))
                else:
                    rospy.logwarn('\033[94m[%s]\033[0m rendering gesture cmd [%s], name [%s]...'%(rospy.get_name(),
                        cmd,
                        self.motion_list['neutral'][random.randint(0, len(self.motion_list['neutral']) - 1)]))

            loop_count = 40
        if 'render_speech' in rospy.get_name():
            rospy.loginfo('\033[94m[%s]\033[0m rendering speech [%s]...'%(rospy.get_name(), goal.data))
            loop_count = 40

        while not rospy.is_shutdown():
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
            rospy.loginfo('\033[95m%s\033[0m rendering completed...' % rospy.get_name())
        else:
            rospy.loginfo('\033[95m%s\033[0m rendering canceled...' % rospy.get_name())


if __name__ == '__main__':
    m = FakeMotionRender()
