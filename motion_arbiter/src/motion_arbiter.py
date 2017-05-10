#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib
import re
import Queue
import json
import operator
from threading import Thread

from std_msgs.msg import String, Bool, Empty
from mhri_msgs.msg import Reply
from mhri_msgs.msg import RenderSceneAction, RenderSceneGoal, RenderItemGoal  # , GazeFocusing
from mhri_msgs.srv import ReadData, ReadDataRequest
from mhri_msgs.msg import LogItem

MAX_QUEUE_SIZE = 10
TIME_FOR_CHARACTER = 0.3
SIZE_FOR_CHARACTER = 3


class OverridingType:
    QUEUE = 0
    CROSS = 1
    OVERRIDE = 2

class SceneQueueData:
    sm = {}
    say = {}
    gaze = {}
    pointing = {}
    sound = {}
    expression = {}
    log = ''

    def __str__(self):
        rospy.loginfo('='*10)
        print ' [SM]        : ', self.sm
        print ' [SAY]       : ', self.say
        print ' [GAZE]      : ', self.gaze
        print ' [POINTING]  : ', self.pointing
        print ' [SOUND]     : ', self.sound
        print ' [EXPRESSION]: ', self.expression
        print ' [LOG]       : ', self.log
        rospy.loginfo('-'*10)
        return ''


class MotionArbiter:

    def __init__(self):
        self.is_rendering = False

        rospy.loginfo('\033[91m[%s]\033[0m waiting for bringup social_mind...'%rospy.get_name())
        self.rd_memory = {}
        try:
            rospy.wait_for_service('social_events_memory/read_data')
            self.rd_memory['social_events_memory'] = rospy.ServiceProxy('social_events_memory/read_data', ReadData)
            rospy.wait_for_service('environmental_memory/read_data')
            self.rd_memory['environmental_memory'] = rospy.ServiceProxy('environmental_memory/read_data', ReadData)
            rospy.wait_for_service('system_events_memory/read_data')
            self.rd_memory['system_events_memory'] = rospy.ServiceProxy('system_events_memory/read_data', ReadData)
        except rospy.exceptions.ROSInterruptException as e:
            rospy.logerr(e)
            quit()

        self.renderer_client = actionlib.SimpleActionClient('render_scene', RenderSceneAction)
        rospy.loginfo('\033[91m[%s]\033[0m waiting for motion_renderer to start...'%rospy.get_name())

        try:
            self.renderer_client.wait_for_server()
        except rospy.exceptions.ROSInterruptException as e:
            quit()

        rospy.Subscriber('reply', Reply, self.handle_domain_reply)
        self.pub_log_item = rospy.Publisher('log', LogItem, queue_size=10)

        self.pub_start_speech_recognizer = rospy.Publisher('speech_recognizer/start', Empty, queue_size=1)
        self.pub_stop_speech_recognizer = rospy.Publisher('speech_recognizer/stop', Empty, queue_size=1)

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

        rospy.loginfo("\033[91m[%s]\033[0m initialized." % rospy.get_name())

    def handle_domain_reply(self, msg):
        scene_item = SceneQueueData()
        recv_msg = msg.reply
        tags = re.findall('(<[^>]+>)', recv_msg)

        scene_item.sm['render'] = 'tag:neutral'
        scene_item.sm['offset'] = 0.0

        scene_item.emotion = {}
        overriding = OverridingType.QUEUE

        for tag in tags:
            tag_msg = recv_msg
            for other_tag in tags:
                if other_tag != tag:
                    tag_msg.replace(other_tag, '')

            index = tag_msg.index(tag)
            recv_msg = recv_msg.replace(tag, '')

            tag = tag.lstrip('<')
            tag = tag.rstrip('>')
            tag = tag.split('=')

            if tag[0].strip() == 'sm':
                scene_item.sm = {}
                scene_item.sm['render'] = tag[1].strip()
                scene_item.sm['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'gaze':
                scene_item.gaze['render'] = tag[1].strip()
                scene_item.gaze['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'pointing':
                scene_item.pointing['render'] = tag[1].strip()
                scene_item.pointing['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'expression':
                scene_item.expression['render'] = tag[1].strip()
                scene_item.expression['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'sound':
                scene_item.sound['render'] = tag[1].strip()
                scene_item.sound['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
            elif tag[0].strip() == 'overriding':
                overriding = int(tag[1].strip())
            elif tag[0].strip() == 'log':
                scene_item.log = tag[1].strip()

        scene_item.say['render'] = recv_msg.strip()
        scene_item.say['offset'] = 0.0

        if scene_item.pointing != {}:
            scene_item.sm = {}

        if overriding == OverridingType.QUEUE:
            self.scene_queue.put(scene_item)
        elif overriding == OverridingType.OVERRIDE:
            if self.is_rendering:
                self.renderer_client.cancel_all_goals()
                rospy.sleep(0.1)
            self.scene_queue.put(scene_item)


    def handle_scene_queue(self):
        rospy.wait_for_service('social_events_memory/read_data')
        rd_memory = rospy.ServiceProxy('social_events_memory/read_data', ReadData)

        while not rospy.is_shutdown():
            if not self.scene_queue.empty():
                goal = RenderSceneGoal()
                scene_dict = {}
                scene_item = self.scene_queue.get()

                # Point and Semantic motion are exclusive relationship. If pointing exists, sm is ignored.
                if scene_item.pointing != {}:
                    target_data = scene_item.pointing['render'].split(':')
                    try:
                        req = ReadDataRequest()
                        req.perception_name = target_data[0]
                        req.query = '{"name": "%s"}'%target_data[1]
                        req.data.append('xyz')
                        req.data.append('frame_id')
                        response = self.rd_memory['environmental_memory'](req)

                        if response.result:
                            rospy.logdebug("read from social_mind for %s: %s"%(target_data[1], response.data))
                            scene_item.pointing['render'] = 'pointing/' + response.data
                            scene_dict['sm'] = scene_item.pointing
                        else:
                            rospy.logwarn("Can't find the information if %s"%target_data[1])
                            scene_dict['sm'] = {'render': 'gesture/tag:neutral', 'offset': scene_item.pointing['offset']}

                    except rospy.ServiceException, e:
                        rospy.logerr("service call failed: %s" % e)
                    except ValueError:
                        scene_dict['sm'] = {'render': 'gesture/tag:neutral', 'offset': scene_item.pointing['offset']}
                else:
                    scene_dict['sm'] = {'render': 'gesture/tag:neutral', 'offset': 0.0}

                if scene_item.gaze != {}:
                    target_data = scene_item.gaze['render'].split(':')
                    try:
                        req = ReadDataRequest()
                        req.perception_name = target_data[0]
                        req.query = '{"name": "%s"}'%target_data[1]
                        req.data.append('xyz')
                        req.data.append('frame_id')
                        response = self.rd_memory['environmental_memory'](req)

                        if response.result:
                            rospy.logdebug("read from social_mind for %s: %s"%(target_data[1], response.data))
                            scene_item.gaze['render'] = response.data
                            scene_dict['gaze'] = scene_item.pointing
                        else:
                            rospy.logwarn("Can't find the information if %s"%target_data[1])

                    except rospy.ServiceException, e:
                        rospy.logerr("service call failed: %s" % e)

                '''
                sm = {}
                say = ''
                gaze = {}
                sound = {}
                expression = {}
                log = ''
                '''

                if scene_item.log != '':
                    msg_log_item = LogItem()
                    log_item = scene_item.log.split('/')
                    for data in log_item:
                        msg_log_item.log_items.append(data)

                    msg_log_item.header.stamp = rospy.Time.now()
                    self.pub_log_item.publish(msg_log_item)


                scene_dict['say'] = scene_item.say
                scene_dict['sound'] = scene_item.sound
                scene_dict['expression'] = scene_item.expression

                # 감정은 소셜 메모리에서 읽어온다.
                scene_dict['emotion'] = {}
                scene_dict['emotion']['current_emotion'] = 'netural'
                scene_dict['emotion']['intensity'] = 1.0

                # if scene.gaze != {}:
                #     gaze_msg = GazeFocusing()
                #     gaze_msg.target_name = motion.gaze_target
                #     gaze_msg.enable = True
                #     self.gazefocus_pub.publish(gaze_msg)

                self.scene_queue.task_done()
                goal.render_scene = json.dumps(scene_dict)
                self.renderer_client.send_goal(goal, done_cb=self.render_done, feedback_cb=self.render_feedback, active_cb=self.render_active)

                while not rospy.is_shutdown() and not self.is_rendering:
                    pass

                while not rospy.is_shutdown() and self.is_rendering:
                    rospy.sleep(0.2)
            else:
                rospy.sleep(0.2)

    def render_active(self):
        rospy.loginfo('\033[91m[%s]\033[0m scene rendering started...'%rospy.get_name())
        self.is_rendering = True
        self.pub_start_speech_recognizer.publish()


    def render_feedback(self, feedback):
        rospy.loginfo('\033[91m[%s]\033[0m scene rendering feedback...'%rospy.get_name())
        pass

    def render_done(self, state, result):
        rospy.loginfo('\033[91m[%s]\033[0m scene rendering done...'%rospy.get_name())
        self.is_rendering = False
        self.pub_stop_speech_recognizer.publish()

        #
        # gaze_msg = GazeFocusing()
        # gaze_msg.target_name = ''
        # gaze_msg.enable = False
        # self.gazefocus_pub.publish(gaze_msg)



if __name__ == '__main__':
    rospy.init_node('motion_arbiter', anonymous=False)
    m = MotionArbiter()
    rospy.spin()
