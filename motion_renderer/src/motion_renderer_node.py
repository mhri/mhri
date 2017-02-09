#!/usr/bin/env python
#-*- encoding: utf8 -*-

import json
import random

import rospy
import actionlib

from mhri_msgs.msg import RenderMotionAction, RenderMotionFeedback, RenderMotionResult
# from mhri_msgs.msg import SpeechActionAction, SpeechActionGoal
# from mhri_msgs.msg import GestureActionAction, GestureActionGoal
# from mhri_msgs.srv import EmptyResult, GetInstalledGestures
# from mhri_msgs.msg import SetFacialExpression

from std_msgs.msg import Bool


class MotionRenderer:
	def __init__(self):
		# self.speech_client = actionlib.SimpleActionClient('speech_action', SpeechActionAction)
		# self.speech_client.wait_for_server()
		#
		# self.gesture_client = actionlib.SimpleActionClient('run_gesture', GestureActionAction)
		# self.gesture_client.wait_for_server()
		#
		# self.pub_face_emotion = rospy.Publisher('set_facial_expression', SetFacialExpression, queue_size=5)
		#
		# rospy.wait_for_service('get_installed_gestures')
		# self.get_motion = rospy.ServiceProxy('get_installed_gestures', GetInstalledGestures)
		# json_data = self.get_motion()
		# self.motion_tag = json.loads(json_data.gestures)
		#
		# rospy.loginfo('[%s] Success to get motion_tag from gesture server'%rospy.get_name())

		self.server = actionlib.SimpleActionServer('render_motion', RenderMotionAction, self.execute_callback, False)
		self.server.register_preempt_callback(self.preempt_callback)
		self.server.start()


		self.is_speaking_now = False
		self.is_playing_now = False
		self.is_gesture_only = False
		self.sync_count_gesture = 0

		rospy.loginfo("[%s] Initialized."%rospy.get_name())


	'''
	Speaking Callback
	'''
	def speech_done_cb(self, state, result):
		self.is_speaking_now = False

	def speech_speaking_cb(self, result):
		pass

	def speech_start_cb(self):
		self.is_speaking_now = True
		pass


	'''
	Gesture Callback
	'''
	def gesture_done_cb(self, state, result):
		self.is_playing_now = False

	def gesture_playing_cb(self, result):
		pass

	def gesture_start_cb(self):
		self.is_playing_now = True


	'''
	Server
	'''
	def preempt_callback(self):
		rospy.logwarn('Motion Rendering Preempted.')
		if self.is_speaking_now:
			self.speech_client.cancel_all_goals()
		if self.is_playing_now:
			self.gesture_client.cancel_all_goals()


	def execute_callback(self, goal):
		# Do lots of awesome groundbreaking robot stuff here
		rospy.logwarn('Motion Rendering Started.')

		result = MotionRenderResult()


		if goal.emotion == 'neutral':
			self.pub_face_emotion.publish(SetFacialExpression.NEUTRAL, goal.emotion_intensity)
		elif goal.emotion == 'happiness':
			self.pub_face_emotion.publish(SetFacialExpression.HAPPINESS, goal.emotion_intensity)
		elif goal.emotion == 'surprise':
			self.pub_face_emotion.publish(SetFacialExpression.HAPPINESS, goal.emotion_intensity)
		elif goal.emotion == 'anger':
			self.pub_face_emotion.publish(SetFacialExpression.HAPPINESS, goal.emotion_intensity)
		elif goal.emotion == 'sadness':
			self.pub_face_emotion.publish(SetFacialExpression.HAPPINESS, goal.emotion_intensity)
		elif goal.emotion == 'disgust':
			self.pub_face_emotion.publish(SetFacialExpression.HAPPINESS, goal.emotion_intensity)
		elif goal.emotion == 'fear':
			self.pub_face_emotion.publish(SetFacialExpression.HAPPINESS, goal.emotion_intensity)

		if goal.gesture != '':
			# When robot requested play gesture, the idle motion is disabled temporary
			self.is_playing_now = True

			#print goal.gesture
			recv_data = goal.gesture.split(':')
			if recv_data[0] == 'sm':
				#print recv_data
				if recv_data[1] in self.motion_tag:
					gesture_name = self.motion_tag[recv_data[1]][random.randrange(0, len(self.motion_tag[recv_data[1]]))]
				else:
					gesture_name = recv_data[1]
			elif recv_data[0] == 'pm':
				gesture_name = recv_data[1]

			gesture_goal = GestureActionGoal(gesture=gesture_name)
			self.gesture_client.send_goal(goal=gesture_goal, done_cb=self.gesture_done_cb, feedback_cb=self.gesture_playing_cb, active_cb=self.gesture_start_cb)

		#rospy.sleep(2)

		if goal.say != '':
			# When robot is speaking, the speech_recognition is disabled temporary
			self.is_speaking_now = True
			self.is_gesture_only = False
			speech_goal = SpeechActionGoal(say=goal.say)
			self.speech_client.send_goal(goal=speech_goal, done_cb=self.speech_done_cb, feedback_cb=self.speech_speaking_cb, active_cb=self.speech_start_cb)
		else:
			# Gesture Only
			self.is_gesture_only = True


		while self.is_speaking_now or self.is_playing_now:
			#rospy.logwarn('%d %d'%(self.is_speaking_now, self.is_playing_now))

			if self.is_gesture_only:
				rospy.sleep(0.2)
				continue

			if not self.is_speaking_now and self.is_playing_now:
				self.sync_count_gesture += 1
				if self.sync_count_gesture > 3:
					self.gesture_client.cancel_all_goals()
					self.sync_count_gesture = 0

			rospy.sleep(0.2)

		self.pub_face_emotion.publish(SetFacialExpression.PREVIOUS_FACE, 1.0)
		rospy.logwarn('Motion Rendering Completed.')

		result.result = True
		self.server.set_succeeded(result)




if __name__ == '__main__':
	rospy.init_node('motion_renderer', anonymous=False)
	m = MotionRenderer()
	rospy.spin()
