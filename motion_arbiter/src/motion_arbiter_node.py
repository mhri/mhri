#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import re
import Queue
import json
import operator
from threading import Thread

from std_msgs.msg import String, Bool
from mhri_msgs.msg import RenderMotionAction, RenderMotionGoal #, GazeFocusing
from mhri_msgs.srv import EmptyResult
from mhri_msgs.srv import ReadData, ReadDataRequest

class OverridingType:
	QUEUE = 0
	CROSS = 1
	OVERRIDE = 2


class MotionQueueData:
	sm_tag = ''
	gaze_target = ''
	say = ''
	point = ''


class MotionArbiter:
	def __init__(self):
		self.is_rendering = False
		rospy.sleep(1)

		rospy.Subscriber('mhri_dialog/reply', String, self.handle_domain_reply)
		self.renderer_client = actionlib.SimpleActionClient('motion_renderer/render_action', MotionRenderAction)
		self.renderer_client.wait_for_server()

		self.gazefocus_pub = rospy.Publisher('gaze_focusing', GazeFocusing, queue_size=5)

		# rospy.Subscriber('emotion_status', EmotionStatus, self.handle_emotion_status, queue_size=10)
		# self.current_emotion = 'neutral'
		# self.current_emotion_intensity = 1.0

		rospy.wait_for_service('idle_motion/is_ready')
		self.pub_set_idle_motion = rospy.Publisher('idle_motion/set_status', Bool, queue_size=10)

		rospy.wait_for_service('/social_memory/write_data')

		self.motion_queue = Queue.Queue(10)
		self.pub_set_idle_motion.publish(True)

		self.t1 = Thread(target=self.handle_motion_queue)
		self.t1.start()

		rospy.loginfo("[%s] Initialized."%rospy.get_name())





	def handle_motion_queue(self):

		while not rospy.is_shutdown():
			if not self.motion_queue.empty():
				try:
					goal = MotionRenderGoal()
					motion = self.motion_queue.get()

					if motion.point != '':
						rospy.wait_for_service('/social_memory/read_data')
						try:
							rd_memory = rospy.ServiceProxy('social_memory/read_data', ReadData)
							rd_data = ReadDataRequest()
							rd_data.event_name = 'person_identification'
							rd_data.data.append('session_face_id')
							rd_data.data.append('face_pos')

							resp1 = rd_memory(rd_data)

						except rospy.ServiceException, e:
							rospy.logerr("Service call failed: %s"%e)

						rospy.logdebug("Read From Social Memory")

						recv_data = json.loads(resp1.data)
						try:
							pos = recv_data['face_pos'][recv_data['session_face_id'].index(motion.point)]

							if abs(pos[0]) < 0.15:
								goal.gesture = 'pm:' + 'sm_pointing/front'
							elif pos[0] >= 0.15 and pos[0] < 0.5:
								goal.gesture = 'pm:' + 'sm_pointing/right_near'
							elif pos[0] <= -0.15 and pos[0] > -0.5:
								goal.gesture = 'pm:' + 'sm_pointing/left_near'
							elif pos[0] >= 0.5:
								goal.gesture = 'pm:' + 'sm_pointing/right_far'
							elif pos[0] <= -0.5:
								goal.gesture = 'pm:' + 'sm_pointing/left_far'

						except ValueError:
							goal.gesture = 'sm:neutral'

					else:
						goal.gesture = 'sm:' + motion.sm_tag

					goal.emotion = motion.emotion
					goal.emotion_intensity = motion.emotion_intensity
					goal.say = motion.say

					if motion.gaze_target != '':
						gaze_msg = GazeFocusing()
						gaze_msg.target_name = motion.gaze_target
						gaze_msg.enable = True
						self.gazefocus_pub.publish(gaze_msg)

					self.motion_queue.task_done()

					self.renderer_client.send_goal(goal, done_cb=self.render_done, feedback_cb=self.render_feedback, active_cb=self.render_active)
					#rospy.sleep(0.2)
					while self.is_rendering:
						rospy.logdebug('Motion Rendering')
						rospy.sleep(0.2)

				except Queue.Empty:
					continue
			else:
				rospy.logdebug("Motion Empty")
				rospy.sleep(0.2)


	def render_active(self):
		rospy.logdebug('Motion Arbiter START')
		self.is_rendering = True

		try:
			rospy.wait_for_service('speech_recognition/stop', 0.1)
			asr_stop = rospy.ServiceProxy('speech_recognition/stop', EmptyResult)
			asr_stop()
		except rospy.ROSException:
			pass


	def render_feedback(self, feedback):
		rospy.logdebug('Motion Arbiter FEEDBACK')
		pass


	def render_done(self, state, result):
		rospy.logdebug('Motion Arbiter EXIT')
		self.is_rendering = False

		try:
			rospy.wait_for_service('speech_recognition/start', 0.1)
			asr_start = rospy.ServiceProxy('speech_recognition/start', EmptyResult)
			asr_start()
		except rospy.ROSException:
			pass

		gaze_msg = GazeFocusing()
		gaze_msg.target_name = ''
		gaze_msg.enable = False
		self.gazefocus_pub.publish(gaze_msg)


	def handle_domain_reply(self, msg):
		goal = MotionQueueData()
		goal.sm_tag = 'neutral'
		goal.gaze_target = ''
		goal.emotion = ''
		goal.emotion_intensity = 0.0
		goal.point = ''
		overriding = 0

		'''
		Example: "<gaze=user_name><sm=attention>Hello! World?"
		'''
		recv_msg = msg.data
		tags = re.findall('(<[^>]+>)', recv_msg)
		for tag in tags:
			recv_msg = recv_msg.replace(tag, '')

			tag = tag.lstrip('<')
			tag = tag.rstrip('>')
			tag = tag.split('=')

			if tag[0].strip() == 'sm':
				goal.sm_tag = tag[1].strip()
			elif tag[0].strip() == 'gaze':
				goal.gaze_target = tag[1].strip()
			elif tag[0].strip() == 'point':
				goal.point = tag[1].strip()
			elif tag[0].strip() == 'emotion':
				goal.emotion = tag[1].strip()
				goal.emotion_intensity = 0.8
			elif tag[0].strip() == 'overriding':
				overriding = int(tag[1].strip())

		goal.say = recv_msg

		if goal.point != '':
			goal.sm_tag = ''

		# if goal.emotion == '':
		# 	goal.emotion = self.current_emotion
		# 	goal.emotion_intensity = self.current_emotion_intensity

		if overriding == 0:
			self.motion_queue.put(goal)
		elif overriding == 2:
			if self.is_rendering:
				self.renderer_client.cancel_all_goals()
				rospy.sleep(0.1)

				self.motion_queue.put(goal)

		rospy.logdebug("Motion Queue Saved.")

if __name__ == '__main__':
	rospy.init_node('motion_arbiter_node', anonymous=False)
	m = MotionArbiter()
	rospy.spin()
