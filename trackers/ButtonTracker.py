#!/usr/bin/env python3
import logger_minimal
import time
import rospy
from std_msgs.msg import Bool

'''
'''
class ButtonTracker:
    def __init__(self, testing=False):
        if rospy.get_name() == '/unnamed':
            rospy.init_node('button_tracker', anonymous=True)
        rospy.Subscriber("/serial/button", Bool, self.sensor_update, queue_size=1)
        if testing:
            rospy.Subscriber("/testing/button", Bool, self.test_update, queue_size=10)
       # self.user_state = Focus.Working
        #self.old_user_state = Focus.Working
        self.status_start_time = time.time()
        self.last_input_time = time.time()
        self.last_check_time = time.time()
        self.status_old_time = 0
        self.new_input = False
        #self.baseline_mode = baseline_mode   # if true, then the button is for acknowledging breaks, and nothing is sent to Cozmo
        self.logger = logger_minimal.Logger('button')
        self.logger.log('START')

    def test_update(self, update):
        pass

    def sensor_update(self, btn_sensor):
        self.logger.log('press')
        self.new_input = True
        self.last_input_time = time.time()

    def get_update(self):
        if(self.new_input):
            self.new_input = False
            self.last_check_time = time.time()
            return True
        return False
