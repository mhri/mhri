#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from mhri_msgs.msg import RaisingEvents
from mhri_msgs.srv import ReadData

class GazeNode:
    def __init__(self):
        rospy.init_node('gaze', anonymous=False)

        rospy.loginfo('\033[92m[%s]\033[0m waiting for bringup social_mind...'%rospy.get_name())
        rospy.wait_for_service('environmental_memory/read_data')
        rospy.wait_for_service('social_events_memory/read_data')

        self.rd_memory = {}
        self.rd_memory['environmental_memory'] = rospy.ServiceProxy('environmental_memory/read_data', ReadData)
        self.rd_memory['social_events_memory'] = rospy.ServiceProxy('social_events_memory/read_data', ReadData)

        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raising_events)

        rospy.Timer(rospy.Duration(0.2), self.handle_gaze_controller)
        rospy.loginfo('\033[92m[%s]\033[0m initialized...'%rospy.get_name())
        rospy.spin()

    def handle_raising_events(self, msg):
        pass

    def handle_gaze_controller(self, event):
        pass


if __name__ == '__main__':
    m = GazeNode()
