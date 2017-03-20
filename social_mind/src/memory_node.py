#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import datetime
import json
import yaml
from Queue import Queue


import pymongo
from mhri_msgs.msg import WaitEventAction, WaitEventFeedback, WaitEventResult
from mhri_msgs.msg import RaiseEvents
from mhri_msgs.srv import ReadData, ReadDataResponse, WriteData, WriteDataResponse


class SetQueue(Queue):
    def _init(self, maxsize):
        Queue._init(self, maxsize)
        self.all_items = set()

    def _put(self, item):
        if item not in self.all_items:
            Queue._put(self, item)
            self.all_items.add(item)

    def _get(self):
        item = Queue._get(self)
        self.all_items.remove(item)
        return item


class MemoryNode:
    def __init__(self):
        rospy.init_node('memory_node', anonymous=False)

        while not rospy.is_shutdown():
            try:
                port = rospy.get_param('/social_mind/port')
                host = rospy.get_param('/social_mind/host')
                break
            except KeyError as e:
                rospy.loginfo('Wait for bringup social_mind node...')
                rospy.sleep(1.0)
                continue

        # Connect to MongoDB & select event_memory
        try:
            self.client = pymongo.MongoClient(host, port, serverSelectionTimeoutMS=2000)
            self.client.is_mongos
        except pymongo.errors.ServerSelectionTimeoutError, e:
            rospy.logerr('Error: %s'%e.message)
            quit()

        # Loading events_template
        name_of_events_group = rospy.get_param('~name_events_group')
        events_file = rospy.get_param('~events')
        with open(events_file) as f:
            events_template = yaml.load(f)

        self.db = self.client[name_of_events_group]
        # Performance Setting: Set the database’s profiling level to 0, and slow_ms is 400ms.
        self.db.set_profiling_level(0, 400)

        # Initialize Events Memory
        self.list_events = events_template['memory_templates']
        self.collector = {}
        for i in self.list_events.keys():
            self.collector[i] = self.db[i]

        # Topics & Services & ActionServer
        self.srv_read_data = rospy.Service(
                    '%s/read_data'%rospy.get_name(),
                    ReadData,
                    self.handle_read_data)
        self.srv_write_data = rospy.Service(
                    '%s/write_data'%rospy.get_name(),
                    WriteData,
                    self.handle_write_data)
        self.wait_event_server = actionlib.SimpleActionServer(
                    '%s/wait_event'%rospy.get_name(),
                    WaitEventAction,
                    self.handle_wait_event,
                    auto_start=False)
        self.wait_event_server.start()
        rospy.loginfo('[%s] Initialzed and ready to use...'%rospy.get_name())


    def handle_wait_event(self, goal):
        """ Handle function for wait_event calling by dialog
        """
        d = datetime.datetime.fromtimestamp(rospy.get_time())

        query_result_count = 0;
        while query_result_count == 0 and self.wait_for_event_server.is_active() == True:
            if self.wait_for_event_server.is_preempt_requested():
                result = WaitForEventResult()
                result.result = False
                result.error_msg = 'The client cancel goal.'
                self.wait_for_event_server.set_preempted(result)
                return result

            for i in range(len(goal.event_name)):
                memory_name = goal.event_name[i]
                memory_query = json.loads(goal.query[i])
                memory_query['time'] = {"$gte": d}

                query_result = self.collector[memory_name].find(memory_query)
                query_result_count += query_result.count()

            rospy.sleep(0.2)

        result = WaitEventResult()
        result.result = True
        self.wait_for_event_server.set_succeeded(result)


    def handle_read_data(self, req):
        """ Handle function for reading some data from memory_node
        Args:
            event_name, data
        Returns:
            True or False
            Requested data.
        """
        query_result = {}
        for data in self.collector[str(req.event_name)].find().sort('_id', pymongo.DESCENDING).limit(1):
            query_result = data

        res = ReadDataResponse()
        if query_result == {}:
            res.result = False
            res.data = '{}'
            return res

        del query_result['_id']
        # Convert datetime to timestamp
        query_result['time'] = float(query_result['time'].strftime('%s.%f'))

        res.result = True
        if len(req.data) == 0:
            res.data = json.dumps(query_result)
        elif len(req.data) == 1 and req.data[0] == '':
            res.data = json.dumps(query_result)
        else:
            ret_data = {}
            for item in req.data:
                ret_data[item] = query_result[item]
            res.data = json.dumps(ret_data)

        return res


    def handle_write_data(self, req):
        """ Handle function for writing some data to memory_node
        Args:
            event, data, by
        Returns:
            True or False
        """
        req.event = req.event.replace("'", '"')
        req.data = req.data.replace("'", '"')

        recv_event = json.loads(req.event)
        recv_data = json.loads(req.data)

        write_data = {}
        if self.list_events.has_key(req.event_name):
            write_data = self.list_events[req.event_name].copy()
        else:
            rospy.logerr('Event [%s] is not exists on this Events.'%req.event_name)
            return WriteDataResponse(False)

        write_data['time'] = datetime.datetime.fromtimestamp(rospy.get_time())
        write_data['by'] = req.by

        for key in recv_event:
            if write_data.has_key(key):
                write_data[key] = recv_event[key]
        for key in recv_data:
            if write_data.has_key(key):
                write_data[key] = recv_data[key]

        self.collector[req.event_name].insert_one(write_data)
        return WriteDataResponse(True)


if __name__ == '__main__':
    m = MemoryNode()
    rospy.spin()
