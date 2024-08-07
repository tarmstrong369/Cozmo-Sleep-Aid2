#!/usr/bin/env python3
import logging
import time
import roslibpy as rospy
import os
import pathlib
import configparser
# from std_msgs.msg import Bool


# SedentaryTracker takes in readings from the seat sensor to determine if the user is standing or sitting.
# Non-symmetric buffer time for standing and sitting can be set to "smooth over"
class SedentaryTracker:
    sit_down_switchover_period = 2
    stand_up_switchover_period_short = 2
    stand_up_switchover_period = 10

    def __init__(self, testing=False):
        self.configPath = str(pathlib.Path(__file__).parents[2]) + '/config.ini'
        config = configparser.ConfigParser()
        config.read(self.configPath)
        self.sit_down_switchover_period = config['SetupSettings'].getfloat('SitDownSwitchover')
        self.stand_up_switchover_period_short = config['SetupSettings'].getfloat('StandUpSwitchoverShort')
        self.stand_up_switchover_period = config['SetupSettings'].getfloat('StandUpSwitchover')

        if rospy.get_name() == '/unnamed':
            rospy.init_node('sedentary_tracker', anonymous=True)
        rospy.Subscriber("/bluetooth/chair", Bool, self.report, queue_size=10)

        if testing:
            rospy.Subscriber("/testing/chair", Bool, self.test_update, queue_size=10)

        # Logging: -1 = standing, 0 = transitioning, 1 = sitting
        self.logger = logger_minimal.Logger('sedentary')
        self.logger.log('START')

        self.is_sitting = True
        self.status_start_time = time.time()
        self.status_old_time = 0
        self.status_changing = False
        self.last_reading = None
        self.has_delayed = False
        self.long_stand = False
        if config.has_section("ChairState"):
            chair_state = config['ChairState']
            self.logger.log("Found state info, restarting from last known state...")
            self.is_sitting = chair_state.getboolean('Sitting')
            self.status_start_time = chair_state.getfloat('StatusStartTime')
            self.has_delayed = chair_state.getboolean('Delayed')
            if self.status_start_time > time.time():
                # we've rolled over and should just reset the start time
                self.status_start_time = time.time()

    def save_state(self):
        config = configparser.ConfigParser()
        config.read(self.configPath)
        if not config.has_section('ChairState'):
            config.add_section('ChairState')
        config.set('ChairState','Sitting', str(self.is_sitting))
        config.set('ChairState','StatusStartTime', str(self.status_start_time))
        config.set('ChairState','Delayed', str(self.has_delayed))
        with open(self.configPath, 'w') as configfile:
            config.write(configfile)
        

    def get_sedentary_duration(self):
        if self.is_sitting:
            return time.time() - self.status_start_time
        else:
            return 0

    def reset(self):
        self.is_sitting = True
        self.status_changing = False
        self.status_start_time = time.time()

    def delay(self, delay_time=600):
        if not self.has_delayed:
            self.has_delayed = True
            self.is_sitting = True
            self.status_changing = False
            self.status_start_time = self.status_start_time + delay_time

    def test_update(self, update):
        pass

    # Aggressive switchover, with reversion if false detection occurs
    def report(self, sitting_sensor):
        self.last_reading = sitting_sensor.data
        # If sensor reports sitting....
        if sitting_sensor.data is True:
            # If we're not currently sitting...
            if not self.is_sitting:
                # And the status is not yet changing to sitting...
                if not self.status_changing:
                    # Start the switchover to sitting sequence
                    print('Sitting switchover beginning.')
                    self.logger.log('0')
                    self.is_sitting = True
                    self.status_changing = True
                    self.status_old_time = self.status_start_time
                    self.status_start_time = time.time()
                # And the status is currently changing to sitting...
                else:
                    # Stop the switchover to standing sequence
                    print('Standing switchover halted; still sitting.')
                    self.logger.log('1')
                    self.is_sitting = True
                    self.status_changing = False
                    self.status_start_time = self.status_old_time
                    self.status_old_time = 0
            # If we're currently sitting...
            else:
                # And the status is changing to sitting...
                if self.status_changing:
                    # Continue the switchover; check if it is greater than the switchover period
                    if time.time() - self.status_start_time > self.sit_down_switchover_period:
                        # Switch over; pretend there was no switchover time
                        print('Sitting switchover complete.')
                        self.has_delayed = False
                        self.logger.log('1')
                        self.status_changing = False
                        self.status_old_time = 0
        # If sensor reports no sitting...
        else:
            # And we're currently sitting...
            if self.is_sitting:
                # And the status is not yet changing to standing...
                if not self.status_changing:
                    # Start the switchover to standing sequence
                    print('Standing switchover beginning.')
                    self.logger.log('0')
                    self.is_sitting = False
                    self.status_changing = True
                    self.status_old_time = self.status_start_time
                    self.status_start_time = time.time()
                else:
                    # Stop the switchover to sitting sequence
                    print('Sitting switchover halted; still standing.')
                    self.logger.log('-1')
                    self.is_sitting = False
                    self.status_changing = False
                    self.status_start_time = self.status_old_time
                    self.status_old_time = 0
            # And we're not currently sitting...
            else:
                # And the status is changing to standing...
                if self.status_changing:
                    # Continue the switchover; check if we're in the short period but over the time for that period
                    if time.time() - self.status_start_time > self.stand_up_switchover_period_short and not self.long_stand:
                        print('Standing switchover in progress.')
                        self.long_stand = True
                        self.is_sitting = False
                        self.status_old_time = self.status_start_time
                    # Continue the switchover; check if we're past the longer period and move to full standing
                    elif time.time() - self.status_start_time > self.stand_up_switchover_period:
                        print('Standing reset complete')
                        self.logger.log('-1')
                        self.status_changing = False
                        self.long_stand = False
                        self.status_old_time = 0