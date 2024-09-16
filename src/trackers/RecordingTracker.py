#!/usr/bin/env python3
import logger_minimal
import time
import rospy
from std_msgs.msg import Bool

'''
'''
class RecordingTracker:
    def __init__(self, testing=False):
        if rospy.get_name() == '/unnamed':
            rospy.init_node('recording_tracker', anonymous=True)
        rospy.Subscriber("/recording", Bool, self.sensor_update, queue_size=1)
        if testing:
            rospy.Subscriber("/testing/webcam", Bool, self.test_update, queue_size=1)

        self.is_recording = False


    def sensor_update(self, update):
        self.is_recording = update.data

    def test_update(self, update):
        pass
