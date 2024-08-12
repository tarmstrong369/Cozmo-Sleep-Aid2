#!/usr/bin/env python3
import logger_minimal
import time
import rospy
import configparser
import pathlib
from std_msgs.msg import Bool

'''
'''
class KeyActivityTracker:
    timeout_period = 2*60   # Time before the system will decide activity has stopped, in seconds

    def __init__(self, testing=False):
        self.configPath = str(pathlib.Path(__file__).parents[2]) + '/config.ini'
        config = configparser.ConfigParser()
        config.read(self.configPath)
        self.timeout_period = config['SetupSettings'].getfloat('KeyTimeout')
        self.timeout_period = self.timeout_period*60    # convert to seconds


        if rospy.get_name() == '/unnamed':
            rospy.init_node('key_tracker', anonymous=True)
        rospy.Subscriber("/serial/keyboard", Bool, self.sensor_update, queue_size=100)
        if testing:
            rospy.Subscriber("/testing/keyboard", Bool, self.test_update, queue_size=10)

        self.logger = logger_minimal.Logger('key_activity')
        self.logger.log('START')

        self.active = True
        self.status_start_time = time.time()
        self.last_input_time = time.time()

        if config.has_section("KeyState"):
            sensor_state = config['KeyState']
            self.logger.log("Found state info, restarting from last known state...")
            self.active = sensor_state.getboolean('Active')
            self.status_start_time = sensor_state.getfloat('StatusStartTime')
            self.last_input_time = sensor_state.getfloat('LastInputTime')

    def save_state(self):
        config = configparser.ConfigParser()
        config.read(self.configPath)
        if not config.has_section('KeyState'):
            config.add_section('KeyState')
        config.set('KeyState','Active', str(self.active))
        config.set('KeyState','StatusStartTime', str(self.status_start_time))
        config.set('KeyState','LastInputTime', str(self.last_input_time))
        with open(self.configPath, 'w') as configfile:
            config.write(configfile)

    def sensor_update(self, btn_sensor):
        if(not self.active):
            self.status_start_time = time.time()
            self.active = True
            self.logger.log('state,active')
        self.logger.log('press')
        self.last_input_time = time.time()
    
    def test_update(self, update):
        pass

    def is_timeout(self):
        return (time.time() - self.last_input_time) > self.timeout_period

    def get_state(self):
        if(self.active and self.is_timeout()):
            self.active = False
            self.status_start_time = time.time()
            self.logger.log('state,inactive')
        return self.status_start_time, self.active
