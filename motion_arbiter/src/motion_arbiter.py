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
from mhri_msgs.msg import Reply
from mhri_msgs.msg import RenderSceneAction, RenderSceneGoal  # , GazeFocusing
# from mhri_msgs.srv import EmptyResult
from mhri_msgs.srv import ReadData, ReadDataRequest

MAX_QUEUE_SIZE = 10
TIME_FOR_CHARACTER = 0.2
SIZE_FOR_CHARACTER = 3


class OverridingType:
    QUEUE = 0
    CROSS = 1
    OVERRIDE = 2


class SceneQueueData:
    # Type: sm = {'tag:sm_motion', 0.0, 'happiness'}
    sm = {}
    say = ''
    gaze = {}
    pointing = {}
    sound = {}
    expression = {}
    emotion = ''
    overriding = 0

    def __str__(self):
        print self.sm
        print self.say
        print self.gaze
        print self.pointing
        print self.sound
        print self.expression
        print self.emotion
        print self.overriding
        return ''


class MotionArbiter:

    def __init__(self):
        self.is_rendering = False

        rospy.loginfo('Waiting for bringup social_memory...')
        try:
            rospy.wait_for_service('/social_memory/write_data')
        except rospy.exceptions.ROSInterruptException as e:
            rospy.logerr(e)
            quit()

        self.renderer_client = actionlib.SimpleActionClient('render_scene', RenderSceneAction)
        rospy.loginfo('Waiting for action server to start...')

        try:
            self.renderer_client.wait_for_server()
        except rospy.exceptions.ROSInterruptException as e:
            quit()

        rospy.Subscriber('reply', Reply, self.handle_domain_reply)

        # self.gazefocus_pub = rospy.Publisher('gaze_focusing', GazeFocusing, queue_size=5)

        # rospy.Subscriber('emotion_status', EmotionStatus, self.handle_emotion_status, queue_size=10)
        # self.current_emotion = 'neutral'
        # self.current_emotion_intensity = 1.0

        # rospy.wait_for_service('idle_motion/is_ready')
        # self.pub_set_idle_motion = rospy.Publisher('idle_motion/set_status', Bool, queue_size=10)

        #

        # self.pub_set_idle_motion.publish(True)

        self.scene_queue = Queue.Queue(MAX_QUEUE_SIZE)

        self.scene_handle_thread = Thread(target=self.handle_scene_queue)
        self.scene_handle_thread.start()

        rospy.loginfo("[%s] Initialized." % rospy.get_name())

    def handle_domain_reply(self, msg):
        scene_item = SceneQueueData()
        recv_msg = msg.reply
        tags = re.findall('(<[^>]+>)', recv_msg)

        scene_item.sm['tag:neutral'] = 0.0
        scene_item.emotion = {}
        scene_item.overriding = 0

        for tag in tags:
            index = recv_msg.index(tag)
            recv_msg = recv_msg.replace(tag, '')

            tag = tag.lstrip('<')
            tag = tag.rstrip('>')
            tag = tag.split('=')

            if tag[0].strip() == 'sm':
                scene_item.sm = {}
                scene_item.sm[tag[1].strip()] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'gaze':
                scene_item.gaze[tag[1].strip()] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'pointing':
                scene_item.pointing[tag[1].strip()] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'expression':
                scene_item.expression[tag[1].strip()] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'sound':
                scene_item.expression[tag[1].strip()] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'emotion':
                if ':' in tag[1].strip():
                    scene_item.emotion[tag[1].strip().split(':')[0]] = float(tag[1].strip().split(':')[1])
            elif tag[0].strip() == 'overriding':
                overriding = int(tag[1].strip())

        scene_item.say = recv_msg.strip()

        if scene_item.pointing != {}:
            scene_item.sm = {}

        # 감정은 소셜 메모리에서 읽어온다.
        if scene_item.emotion == {}:
            scene_item.emotion = {'netural': 0.8}

        print scene_item

        '''
        if overriding == 0:
            self.motion_queue.put(goal)
        elif overriding == 2:
            if self.is_rendering:
                self.renderer_client.cancel_all_goals()
                rospy.sleep(0.1)

                self.motion_queue.put(goal)

        rospy.logdebug("Motion Queue Saved.")
        '''

    def handle_scene_queue(self):
        while not rospy.is_shutdown():

			# Handling the scene_queue that received from domain...
            if not self.scene_queue.empty():
                try:
                    goal = RenderSceneGoal()
                    scene = self.scene_queue.get()

                    if motion.point != '':
                        rospy.wait_for_service('/social_memory/read_data')
                        try:
                            rd_memory = rospy.ServiceProxy(
                                'social_memory/read_data', ReadData)
                            rd_data = ReadDataRequest()
                            rd_data.event_name = 'person_identification'
                            rd_data.data.append('session_face_id')
                            rd_data.data.append('face_pos')

                            resp1 = rd_memory(rd_data)

                        except rospy.ServiceException, e:
                            rospy.logerr("Service call failed: %s" % e)

                        rospy.logdebug("Read From Social Memory")

                        recv_data = json.loads(resp1.data)
                        try:
                            pos = recv_data['face_pos'][
                                recv_data['session_face_id'].index(motion.point)]

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

                    self.renderer_client.send_goal(
                        goal, done_cb=self.render_done, feedback_cb=self.render_feedback, active_cb=self.render_active)
                    # rospy.sleep(0.2)
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
            asr_stop = rospy.ServiceProxy(
                'speech_recognition/stop', EmptyResult)
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
            asr_start = rospy.ServiceProxy(
                'speech_recognition/start', EmptyResult)
            asr_start()
        except rospy.ROSException:
            pass

        gaze_msg = GazeFocusing()
        gaze_msg.target_name = ''
        gaze_msg.enable = False
        self.gazefocus_pub.publish(gaze_msg)



if __name__ == '__main__':
    rospy.init_node('motion_arbiter', anonymous=False)
    m = MotionArbiter()
    rospy.spin()
