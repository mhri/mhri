#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
from mhri_msgs.msg import NotifyEvents
from mhri_msgs.srv import WriteData, ReadData

class PerceptionBase(object):
    def __init__(self):
        rospy.loginfo('[%s] Wait for bringup social_events_memory...'%rospy.get_name())
        rospy.wait_for_service('/social_events_memory/write_data')
        self.srv_social_events_wr_ = rospy.ServiceProxy('/social_events_memory/write_data', WriteData)
        self.srv_social_events_rd_ = rospy.ServiceProxy('/social_events_memory/read_data', ReadData)

        rospy.loginfo('[%s] Wait for bringup environmental_memory...'%rospy.get_name())
        rospy.wait_for_service('/environmental_memory/write_data')
        self.srv_environmental_wr_= rospy.ServiceProxy('/environmental_memory/write_data', WriteData)
        self.srv_environmental_rd_ = rospy.ServiceProxy('/environmental_memory/read_data', ReadData)

        rospy.loginfo('[%s] Wait for bringup system_events_memory...'%rospy.get_name())
        rospy.wait_for_service('/system_events_memory/write_data')
        self.srv_system_events_wr_ = rospy.ServiceProxy('/system_events_memory/write_data', WriteData)
        self.srv_system_events_rd_ = rospy.ServiceProxy('/system_events_memory/read_data', ReadData)

        self.pub_events = rospy.Publisher('notify_events', NotifyEvents, queue_size=10)

    def notify_events(self, events):
        print "TEST"
        pass

    def save_to_memory(self, target_memory, data):
        pass

    def read_from_memory(self, target_memory, data):
        pass
