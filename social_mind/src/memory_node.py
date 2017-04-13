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
from mhri_msgs.msg import RaisingEvents
from mhri_msgs.srv import ReadData, ReadDataResponse, WriteData, WriteDataResponse, RegisterData, RegisterDataResponse, GetDataList, GetDataListResponse


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

        try:
            self.client = pymongo.MongoClient(host, port, serverSelectionTimeoutMS=2000)
            self.client.is_mongos   # Wait for connection
            self.db = self.client[rospy.get_param('~name_data_group')]
            # self.db.set_profiling_level(0, 400) # Performance Setting: Set the database’s profiling level to 0, and slow_ms is 400ms.
            self.collections = {}
            self.data_template = {}
        except pymongo.errors.ServerSelectionTimeoutError, e:
            rospy.logerr('Error: %s'%e.message)
            quit()
        except KeyError, e:
            quit()

        self.srv_read_data = rospy.Service('%s/read_data'%rospy.get_name(), ReadData, self.handle_read_data)
        self.srv_write_data = rospy.Service('%s/write_data'%rospy.get_name(), WriteData, self.handle_write_data)
        self.srv_register_data = rospy.Service('%s/register_data'%rospy.get_name(), RegisterData, self.handle_register_data)
        self.srv_get_data_list = rospy.Service('%s/get_data_list'%rospy.get_name(), GetDataList, self.handle_get_data_list)

        self.wait_event_server = actionlib.SimpleActionServer('%s/wait_event'%rospy.get_name(), WaitEventAction, self.handle_wait_event, auto_start=False)
        self.wait_event_server.start()
        rospy.loginfo('[%s] Initialzed and ready to use...'%rospy.get_name())

    def handle_wait_event(self, goal):
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
        query_result = {}
        for data in self.collections[str(req.perception_name)].find().sort('_id', pymongo.DESCENDING).limit(1):
            query_result = data

        res = ReadDataResponse()
        if query_result == {}:
            res.result = False
            res.data = '{}'
            return res

        del query_result['_id']
        query_result['time'] = float(query_result['time'].strftime('%s.%f')) # Convert datetime to timestamp

        res.result = True
        if len(req.query) == 0:
            res.data = json.dumps(query_result)
        elif len(req.query) == 1 and req.query[0] == '':
            res.data = json.dumps(query_result)
        else:
            ret_data = {}
            for item in req.query:
                ret_data[item] = query_result[item]
            res.data = json.dumps(ret_data)
        return res

    def handle_write_data(self, req):
        recv_data = json.loads(req.data)
        write_data = recv_data.copy()
        write_data['time'] = datetime.datetime.fromtimestamp(rospy.get_time())
        write_data['by'] = req.by

        self.collections[req.perception_name].insert_one(write_data)
        return WriteDataResponse(True)

    def handle_register_data(self, req):
        self.data_template[req.perception_name] = json.loads(req.data)
        self.collections[req.perception_name] = self.db[req.perception_name]
        return RegisterDataResponse(True)

    def handle_get_data_list(self, req):
        resp = GetDataListResponse()
        resp.result = True
        resp.data_list = json.dumps(self.data_template)
        return resp


if __name__ == '__main__':
    m = MemoryNode()
    rospy.spin()
