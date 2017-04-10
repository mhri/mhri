#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import yaml
import json
import rospy
from mhri_msgs.msg import ForwardingEvent
from mhri_msgs.srv import WriteData, WriteDataRequest, ReadData, RegisterData, RegisterDataRequest


class PerceptionBase(object):
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=False)

        try:
            conf_file = rospy.get_param('~config_file')
        except KeyError as e:
            rospy.logerr('You should set the parameter for perception config file...')
            quit()

        with open(os.path.abspath(conf_file)) as f:
            self.conf_data = yaml.load(f.read())
            rospy.loginfo('loading perception config file... %d perception(s) exists...'%len(self.conf_data.keys()))
            for item in self.conf_data.keys():
                rospy.loginfo('\033[92m  - %s: %d event(s) and %d data(s).\033[0m'%(item, len(self.conf_data[item]['events']), len(self.conf_data[item]['data'])))

        self.dict_srv_wr = {}
        self.dict_srv_rd = {}

        for item in self.conf_data.keys():
            if self.conf_data[item].has_key('target_memory'):
                memory_name = self.conf_data[item]['target_memory']
                rospy.loginfo('\033[94m[%s]\033[0m Wait for bringup %s...'%(rospy.get_name(), memory_name))

                rospy.wait_for_service('/%s/write_data'%memory_name)
                self.dict_srv_wr[memory_name] = rospy.ServiceProxy('/%s/write_data'%memory_name, WriteData)
                self.dict_srv_rd[memory_name] = rospy.ServiceProxy('/%s/read_data'%memory_name, ReadData)

                self.register_data_to_memory(memory_name, item, self.conf_data[item]['data'])

        self.pub_event = rospy.Publisher('forwarding_event', ForwardingEvent, queue_size=10)
        rospy.loginfo('\033[94m[%s]\033[0m Initialize PerceptionBase done...'%rospy.get_name())


    def raise_event(self, perception_item, event):
        if not perception_item in self.conf_data.keys():
            rospy.logwarn('<%s> perception is not member of perception configuration...'%perception_item)
            return
        if not event in self.conf_data[perception_item]['events']:
            rospy.logwarn('<%s> event is not member of event list of perception configuration...'%event)
            return

        msg = ForwardingEvent()
        msg.header.stamp = rospy.Time.now()
        msg.event = event
        msg.by = perception_item

        self.pub_event.publish(msg)

    def register_data_to_memory(self, memory_name, perception_name, data):
        rospy.wait_for_service('/%s/register_data'%memory_name)
        srv_register = rospy.ServiceProxy('/%s/register_data'%memory_name, RegisterData)

        srv_req = RegisterDataRequest()
        srv_req.perception_name = perception_name
        srv_req.data = json.dumps(data)

        return srv_register(srv_req)

    def save_to_memory(self, perception_name, data={}):
        if data == {}:
            rospy.logwarn('Empty data inserted...')
            return

        for item in data.keys():
            if not item in self.conf_data[perception_name]['data'].keys():
                rospy.logwarn('Wrong data inserted...')
                return

        srv_req = WriteDataRequest()
        srv_req.perception_name = perception_name
        srv_req.data = json.dumps(data)
        srv_req.by = rospy.get_name()

        target_memory = self.conf_data[perception_name]['target_memory']
        self.dict_srv_wr[target_memory](srv_req)

    def read_from_memory(self, target_memory, data):
        pass
