#!/usr/bin/env python3
import logger_minimal
import time
import rospy
import configparser
import pathlib
from std_msgs.msg import Bool
from enum import Enum
from trackers import KeyActivityTracker, ButtonTracker, SedentaryTracker, RecordingTracker
from UserBreakState import UserBreakState

'''
Focus tracker takes the keyboard, button, and fidget sensor data to determine if a user is still working or if they've gotten distracted and could maybe use a break.
'''
class FocusTracker:
    is_focus_break = False
    last_break_ended_time = 1.0
    last_break_start_time = 0.0
    no_key_activity_max_time = 15.0*60.0 # time before Cozmo will say to take a break because distracted in seconds
    snooze_on = False   # are we snoozed?
    key_activity = False    # are you active?
    key_activity_state_start_time = 0.0
    on_break = False    # are they already taking a break
    cozmo_active = False # is Cozmo currently trying to make them take a break
    sedentary_max_time = 60*60*2 # time before a user should stand up n seconds
    sedentary_interrupt_window = 60*30
    snooze_delay = 5.0*60.0 # time before cozmo will start yelling again in seconds
    snooze_start_time = 0.0
    recording_offset = 2*60 # Timeout offset lead up to start recording
    log_message_old = ""
    log_message = ""

    def __init__(self, baseline_mode=False, testing=False):
        self.sedentary_tracker = SedentaryTracker.SedentaryTracker(testing=testing)
        self.button_tracker = ButtonTracker.ButtonTracker(testing=testing)
        self.key_tracker = KeyActivityTracker.KeyActivityTracker(testing=testing)
        self.recording_tracker = RecordingTracker.RecordingTracker(testing=testing)

        if rospy.get_name() == '/unnamed':
            rospy.init_node('focus_tracker', anonymous=True)
        if testing:
            rospy.Subscriber("/testing/focus", String, self.test_update, queue_size=10)
        self.logger = logger_minimal.Logger('focus')
        self.logger.log('START')

        # self.configPath = str(pathlib.Path(__file__).parents[2]) + '/config.ini'
        # config = configparser.ConfigParser()
        # config.read(self.configPath)
        # breaktime = config['SetupSettings'].getfloat('MaxBreakInterruptHr')
        # breakwindow = config['SetupSettings'].getfloat('InterruptWindowHr')
        # snooze_time = config['SetupSettings'].getfloat('SnoozeDelay')
        # record_offset = config['SetupSettings'].getfloat('RecordOffset')

        self.configPath = str(pathlib.Path(__file__).parents[2]) + '/config.ini'
        config = configparser.ConfigParser()
        config.read(self.configPath)
        wakehr = config['SetupSettings'].getfloat('Wakeuphr')
        wakemin = config['SetupSettings'].getfloat('Wakeupmin') 
        bedhr = config['SetupSettings'].getfloat('Bedtimehr')
        bedmin = config['SetupSettings'].getfloat('Bedtimemin')
        snooze_time = config['SetupSettings'].getfloat('SnoozeDelay')
        record_offset = config['SetupSettings'].getfloat('RecordOffset')

        if config.has_section("FocusState"):
            self.logger.log("Found state info, restarting from last known state...")
            self.snooze_on = config['FocusState'].getboolean('SnoozeOn')
            self.snooze_start_time = config['FocusState'].getfloat('SnoozeStartTime')
            #self.cozmo_active = config['FocusState'].getboolean('CozmoActive')
            self.cozmo_active = False # Rather than handle if cozmo was active and the problem was like hours ago
            self.on_break = config['FocusState'].getboolean('Break')
            self.last_break_start_time = config['FocusState'].getfloat('BreakStartTime')
            self.last_break_ended_time = config['FocusState'].getfloat('BreakEndTime')
            if self.last_break_ended_time > time.time():
                # we've rolled over and should just reset
                self.on_break = False
                self.last_break_ended_time = time.time()
            if self.last_break_start_time > time.time():
                # we've rolled over and should just reset
                self.last_break_start_time = 0.0
            if self.snooze_start_time > time.time():
                # we've rolled over and hsould just reset
                self.snooze_start_time = 0.0
                self.snooze_on = False


        self.snooze_delay = 60*snooze_time
        self.recording_offset = 60*record_offset
        self.sedentary_max_time = 60*60*breaktime # time before a user should stand up n seconds
        self.sedentary_interrupt_window = 60*60*breakwindow
        self.baseline_mode = baseline_mode   # if true, then the button is for acknowledging breaks, and nothing is sent to Cozmo
        self.wakeuphr = wakehr
        self.wakeupmin = wakemin
        self.bedtimehr = bedhr
        self.bedtimemin = bedmin




    def save_state(self):
        self.key_tracker.save_state()
        self.sedentary_tracker.save_state()
        config = configparser.ConfigParser()
        config.read(self.configPath)
        if not config.has_section('FocusState'):
            config.add_section('FocusState')
        config.set('FocusState','SnoozeOn', str(self.snooze_on))
        config.set('FocusState','SnoozeStartTime', str(self.snooze_start_time))
        config.set('FocusState','CozmoActive', str(self.cozmo_active))
        config.set('FocusState','OnBreak', str(self.on_break))
        config.set('FocusState','BreakStartTime', str(self.last_break_start_time))
        config.set('FocusState','BreakEndTime', str(self.last_break_ended_time))
        with open(self.configPath, 'w') as configfile:
            config.write(configfile)

    def change_mode(self, baseline_mode):
        if self.baseline_mode == baseline_mode:
            return
        self.baseline_mode = baseline_mode
        self.snooze_on = False
        self.snooze_start_time = 0.0
        self.cozmo_active = False
        self.last_break_start_time = time.time()
        self.on_break = False
        self.last_break_ended_time = time.time()
        self.sedentary_tracker.status_start_time = time.time()
        self.sedentary_tracker.has_delayed = False
        self.log_message_old = ""
        self.log_message = ""

    def test_update(self, update):
        pass

    def get_state(self):
        # Get any button updates
        if(self.button_tracker.get_update()):
            if(self.baseline_mode):
                self.logger.log('call,break,button')
                return UserBreakState.BreakRequest
            elif(not self.snooze_on and self.cozmo_active):
                self.logger.log('call,snooze,button')
                return UserBreakState.SnoozeTriggered
        # ////////////////////////////////MY CODE STARTS HERE/////////////////////////////////////////
        # If cozmo is trying to get the user to wakeup/stand then...
        if(self.cozmo_active):
            if(not self.is_standing()):
                current_time = datetime.now()
                hour = current_time.hour
                minute = current_time.minute
                second = current_time.second
                if self.istime(hour, minute, self.wakeuphr, self.wakeupmin):
                    if(self.snooze_on):
                        # Check if snooze time is up...
                        if(self.get_snooze_over()):
                        # Check if we're still sitting or working...
                            if(not self.is_standing()): # or (self.is_standing() and self.key_activity)):
                                self.log_message = 'call,cozmo,snooze'
                                return_state = UserBreakState.NeedCozmo
                    # If neither of these they probably started their break...
                            else:
                                self.log_message = 'call,break,snooze'
                                return_state = UserBreakState.BreakRequest
                    else:
                        self.log_message = 'call,cozmo,snooze'
                        return_state = UserBreakState.NeedCozmo
                else:
                    self.log_message = 'call,break,snooze'
                    return_state = UserBreakState.BreakRequest
            # If cozmo is trying to get the user to go to bed/sit then...
            if(self.is_standing()):
                current_time = datetime.now()
                hour = current_time.hour
                minute = current_time.minute
                second = current_time.second
                if self.istime(hour, minute, self.bedtimehr, self.bedtimemin):
                    if(self.snooze_on):
                        # Check if snooze time is up...
                        if(self.get_snooze_over()):
                        # Check if we're still sitting or working...
                            if(self.is_standing()): # or (self.is_standing() and self.key_activity)):
                                self.log_message = 'call,cozmo,snooze'
                                return_state = UserBreakState.NeedCozmo
                    # If neither of these they probably started their break...
                            else:
                                self.log_message = 'call,break,snooze'
                                return_state = UserBreakState.BreakRequest
                    else:
                        self.log_message = 'call,cozmo,snooze'
                        return_state = UserBreakState.NeedCozmo
                else:
                    self.log_message = 'call,break,snooze'
                    return_state = UserBreakState.BreakRequest



# //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        # If cozmo is trying to get the user to take a break then...
        if(self.cozmo_active):
            # if we're still sitting and have not reached short stand....
            if(not self.is_short_stand()):
                self.sedentary_tracker.has_delayed = False
                self.log_message = 'call,cozmo,ongoing'
                return_state = UserBreakState.NeedCozmo
      
            else:
                self.log_message = 'call,break,cozmo'
                return_state = UserBreakState.BreakRequest
        # If we're on break...
        elif(self.on_break):
            # If the user sits down then the break is over
            if(self.is_sitting()):
                self.break_ended()
                self.log_message = 'stop,break,sitting'
                return_state = UserBreakState.BreakEnded
            else:
                self.log_message = 'ongoing,break'
                return_state = UserBreakState.OnBreak
        # If we're not already on a break...
        elif(not self.on_break):
            # If we've been on snooze...
            if(self.snooze_on):
                # Check if snooze time is up...
                if(self.get_snooze_over()):
                    # Check if we're still sitting or working...
                    if(not self.is_standing()): # or (self.is_standing() and self.key_activity)):
                        self.log_message = 'call,cozmo,snooze'
                        return_state = UserBreakState.NeedCozmo
                    # If neither of these they probably started their break...
                    else:
                        self.log_message = 'call,break,snooze'
                        return_state = UserBreakState.BreakRequest
                # Even if snooze isn't over... let's reset it if they've started their break
                elif(self.is_standing()): # and not self.key_activity):
                    self.log_message = 'call,break,snooze'
                    return_state = UserBreakState.BreakRequest
                # If snooze time isn't up then don't worry about it
                else:
                    self.log_message = 'ongoing,snooze'
                    return_state = UserBreakState.NoBreak
            # If we haven't been on snooze...
            else:
                # We're sitting or standing and working and...
                if(not self.is_standing()): # or (self.is_standing() and self.key_activity)):
                    # we're within the break window and the user is not actively typing...
                    if(self.can_interrupt() and not self.key_activity):
                        self.sedentary_tracker.has_delayed = False
                        self.log_message = 'call,cozmo,sitting'
                        return_state = UserBreakState.NeedCozmo
                    # we've been sitting TOO LONG
                    elif(self.sedentary_tracker.get_sedentary_duration() > self.sedentary_max_time):
                        self.sedentary_tracker.has_delayed = False
                        self.log_message = 'call,cozmo,sitting,max'
                        return_state = UserBreakState.NeedCozmo
                    else:
                        self.log_message = 'ongoing,working'
                        return_state = UserBreakState.NoBreak
                # we're not sitting then...
                else:
                    self.log_message = 'call,break,standing'
                    return_state = UserBreakState.BreakRequest

        # If we don't meet any of the break stuff then no break is needed
        else:
            self.log_message = 'none'
            return_state = UserBreakState.NoBreak
        
        # If there is a new log message, log and print
        if self.log_message != self.log_message_old:
            self.log_message_old = self.log_message
            self.logger.log(self.log_message)

        return return_state


    def istime(hr,min,targhr,targmin,margin=1):
        current_time = datetime.now()
        target_time = datetime(current_time.year, current_time.month, current_time.day, targhr, targmin)
        return abs((current_time - target_time).total_seconds()) < margin
    
    def start_cozmo(self):
        if(not self.cozmo_active):
            self.cozmo_active = True
            self.snooze_on = False
            self.logger.log('start,cozmo')

    def cancel_break(self):
        self.cozmo_active = False
        self.on_break = False
        self.snooze_on = False
        self.is_focus_break = False
        self.last_break_ended_time = time.time()
        self.logger.log('stop,cozmo')

    def start_snooze(self):
        self.cozmo_active = False
        self.snooze_on = True
        self.snooze_start_time = time.time()
        #self.sedentary_tracker.delay(self.snooze_delay)
        self.logger.log('start,snooze')

    def break_started(self):
        self.on_break = True
        self.cozmo_active = False
        self.snooze_on = False
        self.is_focus_break = False
        self.last_break_start_time = time.time()
        self.logger.log('start,break')

    def break_ended(self):
        self.on_break = False
        self.cozmo_active = False
        self.snooze_on = False
        self.is_focus_break = False
        self.last_break_ended_time = time.time()



    def get_snooze_over(self):
        if(self.snooze_on and (time.time() - self.snooze_start_time) > self.snooze_delay):
            self.logger.log('stop,snooze,timer')
            self.snooze_on = False
        return not self.snooze_on

    def is_standing(self):
        return not self.sedentary_tracker.is_sitting and not self.sedentary_tracker.status_changing

    def is_short_stand(self):
        return not self.sedentary_tracker.is_sitting
    
    def is_sitting(self):
        return self.sedentary_tracker.is_sitting and not self.sedentary_tracker.status_changing

    def can_interrupt(self):
        return (self.sedentary_tracker.get_sedentary_duration() > (self.sedentary_max_time - self.sedentary_interrupt_window))





