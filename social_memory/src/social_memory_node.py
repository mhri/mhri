#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import datetime
import json
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

class main:
	def __init__(self):

		rospy.init_node('memory_monitor', anonymous=False)
		rospy.loginfo('Wait for bringup social_warehouse node...')

		# Connect to MongoDB & select event_memory
		port = rospy.get_param('social_warehouse/port', 33402)
		host = rospy.get_param('social_warehouse/host', 'localhost')

		try:
			self.client = pymongo.MongoClient(host, port, serverSelectionTimeoutMS=2000)
			self.client.is_mongos
		except pymongo.errors.ServerSelectionTimeoutError, e:
			rospy.logerr('Error: %s'%e.message)
			quit()

		self.social_db = self.client['social_events']
		self.system_db = self.client['system_events']



		# Set the database’s profiling level to 0, and slow_ms is 400ms.
		self.social_db.set_profiling_level(0, 400)
		self.system_db.set_profiling_level(0, 400)

		self.list_social_events = rospy.get_param('~social_events')
		self.list_system_events = rospy.get_param('~system_events')

		self.social_collector = {}
		self.system_collector = {}
		for i in self.list_social_events.keys():
			self.social_collector[i] = self.social_db[i]
		for i in self.list_system_events.keys():
			self.system_collector[i] = self.system_db[i]



		event_period = rospy.get_param('~event_period', 0.5)

		# Topics & Services & ActionServer
		self.pub_raise_events = rospy.Publisher('social_memory/raise_events', RaiseEvents, queue_size=1)
		self.srv_read_data = rospy.Service('social_memory/read_data', ReadData, self.handle_read_data)
		self.srv_write_data = rospy.Service('social_memory/write_data', WriteData, self.handle_write_data)
		self.wait_event_server = actionlib.SimpleActionServer('social_memory/wait_event', WaitEventAction, self.handle_wait_event, auto_start=False)
		self.wait_event_server.start()

		# Queue for Events
		self.event_queue = SetQueue()
		self.recognized_word_queue = SetQueue()

		rospy.Timer(rospy.Duration(event_period), self.handle_event_trigger)
		rospy.loginfo('[%s] Initialzed and ready to use...'%rospy.get_name())


	def handle_event_trigger(self, event):
		if self.event_queue.empty() and self.recognized_word_queue.empty():
			return

		data = RaiseEvents()
		data.header.stamp = rospy.Time.now()

		if not self.recognized_word_queue.empty():
			data.recognized_word = self.recognized_word_queue.get()
		if data.recognized_word != '':
			self.recognized_word_queue.task_done()

		while not self.event_queue.empty():
			data.events.append(self.event_queue.get())
		if len(data.events) > 0:
			self.event_queue.task_done()

		self.pub_raise_events.publish(data)


	def handle_wait_event(self, goal):
		# Check Event
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

				query_result = self.social_collector[memory_name].find(memory_query)
				query_result_count += query_result.count()

			rospy.sleep(0.2)

		result = WaitEventResult()
		result.result = True
		self.wait_for_event_server.set_succeeded(result)


	'''
	[Example] ReadData
	---
	event_name: speech_recognition
	data: ['recognized_word', 'confidence']
	---
	result: true or false
	data: '{"recognized_word": "Hello World", "confidence": 0.95}'
	'''
	def handle_read_data(self, req):
		query_result = {}
		for data in self.social_collector[str(req.event_name)].find().sort('_id', pymongo.DESCENDING).limit(1):
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

	'''
	[Example] WriteData
	---
	event_name: speech_recognition
	event: '{"speech_recognized": true}'
	data:  '{"recognized_word": "Hello World", "confidence": 0.95}'
	by: "nasr_speech_recognizer"
	---
	result: true or false
	'''
	def handle_write_data(self, req):
		req.event = req.event.replace("'", '"')
		req.data = req.data.replace("'", '"')

		recv_event = json.loads(req.event)
		recv_data = json.loads(req.data)


		write_data = {}
		if self.list_social_events.has_key(req.event_name):
			write_data = self.list_social_events[req.event_name].copy()
		else:
			rospy.logerr('Event [%s] is not exists on Social Events.'%req.event_name)
			return WriteDataResponse(False)

		write_data['time'] = datetime.datetime.fromtimestamp(rospy.get_time())
		write_data['by'] = req.by

		for key in recv_event:
			if write_data.has_key(key):
				write_data[key] = recv_event[key]
		for key in recv_data:
			if write_data.has_key(key):
				write_data[key] = recv_data[key]

		self.social_collector[req.event_name].insert(write_data, safe=False)

		if req.event_name == 'speech_recognition':
			if write_data['recognized_word'] != '':
				self.recognized_word_queue.put(write_data['recognized_word'])

		for k in recv_event.keys():
			if k != 'speed_recognized':
				self.event_queue.put(k)

		return WriteDataResponse(True)


if __name__ == '__main__':
	m = main()
	rospy.spin()
