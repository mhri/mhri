#!/usr/bin/python
#-*- encoding: utf8 -*-

import random
import rospy
from threading import Lock

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from mhri_msgs.msg import RaisingEvents
from mhri_msgs.srv import ReadData, ReadDataRequest


GAZE_CONTROLLER_PERIOD = 0.2
GLANCE_TIMEOUT_MEAN = 3.0

class GazeState:
    IDLE = 0
    FOCUSING = 1
    TRACKING = 2
    GLANCE = 3
    UNKNOWN = -1

class GazeNode:
    def __init__(self):
        rospy.init_node('gaze', anonymous=False)

        self.lock = Lock()
        self.current_state = GazeState.IDLE

        # Initialize Variables
        self.glance_timeout = random.randrange(
            GLANCE_TIMEOUT_MEAN/GAZE_CONTROLLER_PERIOD, (GLANCE_TIMEOUT_MEAN+1.0)/GAZE_CONTROLLER_PERIOD)
        self.glance_timecount = 0
        self.glace_played = False

        rospy.loginfo('\033[92m[%s]\033[0m waiting for bringup social_mind...'%rospy.get_name())
        rospy.wait_for_service('environmental_memory/read_data')
        rospy.wait_for_service('social_events_memory/read_data')

        self.rd_memory = {}
        self.rd_memory['environmental_memory'] = rospy.ServiceProxy('environmental_memory/read_data', ReadData)
        self.rd_memory['social_events_memory'] = rospy.ServiceProxy('social_events_memory/read_data', ReadData)

        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raising_events)
        rospy.Subscriber('gaze_focusing', String, self.handle_gaze_focusing)

        rospy.Timer(rospy.Duration(GAZE_CONTROLLER_PERIOD), self.handle_gaze_controller)
        rospy.loginfo('\033[92m[%s]\033[0m initialized...'%rospy.get_name())
        rospy.spin()

    def handle_raising_events(self, msg):
        if len(msg.events) == 0:
            return

        # Event가 전달되면, 게이즈에 해당되는 이벤트가 포함되는지 확인해서 있으면 모드에 맞게 변경
        self.lock.acquire()
        if 'loud_sound_detected' in msg.events:
            self.last_state = self.current_state
            self.current_state = GazeState.GLANCE
        elif 'person_appeared' in msg.events:
            self.last_state = self.current_state
            self.current_state = GazeState.TRACKING
        self.lock.release()


    def handle_gaze_focusing(self, msg):
        # 환경 메모리에서 전달된 이름에 대한 정보가 있는지 확인해보고, 있으면 타겟설정, 없으면 현재모드 유지
        pass

    def handle_gaze_controller(self, event):
        # 0.2ms (조정가능) 주기로 동작되는 컨트롤러 모드에 따라 동작을 달리한다.
        if self.current_state == GazeState.IDLE:
            print "IDLE"

        elif self.current_state == GazeState.GLANCE:
            if not self.glace_played:
                req = ReadDataRequest()
                req.perception_name = 'loud_sound_detection'
                req.query = '{}'
                req.data.append('xyz')
                req.data.append('frame_id')
                response = self.rd_memory['social_events_memory'](req)

                if not response.result:
                    self.current_state = self.last_state
                    return

                target = PointStamped()
                target.header.frame_id = 'base_footprint'

                target.point.x = 1.0
                target.point.z = 0.6 + (random.randrange(0, 30) / 100.0)
                if response.data['xyz'][1] < -0.2:   #Right Side
                    target.point.y = -1.0 * random.randrange(10, 20) / 10.0
                elif response.data['xyz'][1] > 0.2:   #Left Side
                    target.point.y = random.randrange(10, 20) / 10.0
                else:
                    target.point.y = 0

                # Publish (target)
                rospy.loginfo('\033[92m[%s]\033[0m changed the state - [GLANCE]...'%rospy.get_name())
                self.glace_played = True
            else:
                self.glance_timecount += 1
                if self.glance_timecount > self.glance_timeout:
                    self.glace_played = False
                    self.glance_timecount = 0

                    self.lock.acquire()
                    self.current_state = self.last_state
                    self.lock.release()
                    rospy.loginfo('\033[92m[%s]\033[0m return from GLANCE to last state...'%rospy.get_name())

        elif self.current_state == GazeState.FOCUSING:
            print "FOCUSING"

        elif self.current_state == GazeState.TRACKING:
            print "TRACKING"


if __name__ == '__main__':
    m = GazeNode()
