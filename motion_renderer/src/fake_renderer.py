#!/usr/bin/env python
#-*- encoding: utf8 -*-

import json
import rospy
import actionlib

from mhri_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback


class FakeMotionRender:

    def __init__(self):
        rospy.init_node('fake_renderer', anonymous=False)

        try:
            topic_name = rospy.get_param('~topic_name')
        except KeyError as e:
            print('[ERROR] Needed parameter for (node_name, topic_name)...')
            quit()

        self.server = actionlib.SimpleActionServer(
            topic_name, RenderItemAction, self.execute_callback, False)
        self.server.start()

        rospy.loginfo('[%s] initialized...' % rospy.get_name())
        rospy.spin()

    def execute_callback(self, goal):
        rospy.loginfo('%s rendering requested...' % rospy.get_name())
        result = RenderItemResult()
        feedback = RenderItemFeedback()

        success = True

        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                success = False
                break

            feedback.is_rendering = True
            self.server.publish_feedback(feedback)
            rospy.sleep(0.1)

        if success:
            result.result = True
            self.server.set_succeeded(result)
            rospy.loginfo('%s rendering completed...' % rospy.get_name())
        else:
            rospy.loginfo('%s rendering canceled...' % rospy.get_name())


if __name__ == '__main__':
    m = FakeMotionRender()
