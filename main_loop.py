#!/usr/bin/env python3
import rospy
import configparser
import logger_minimal
import sys
import os
import pathlib
import time
import datetime
import signal
from std_msgs.msg import Bool
import traceback
import cozmo_controller
import subprocess
import logging
#import webcam_recorder
from trackers import SedentaryTracker, FocusTracker
from UserBreakState import UserBreakState

def cozmo_loop(testing=False):
    # Try to connect to Cozmo...
    connect_code = cozmo.connect()
    # If connection failed...
    if connect_code != 0:
        if connect_code == -1:
            # Exception occurred, try reconnect
            #logger.log("Connection Failed")
            return
        elif connect_code == 1:
            # Keyboard interrupt, raise system quit
            shutdown_callback(1,1)
            return
        elif connect_code == 2:
            # Firmware updated successfully, try reconnect
            return
        elif connect_code ==3:
            # firmware update failed, try again here then try reconnect
            subprocess.call(["pycozmo_update.py", "/home/cozmo/.pycozmo/assets/cozmo_resources/config/engine/firmware/cozmo.safe"])
            return
    
    log_message = ""
    log_message_old = ""
    last_state_save = time.time()
    mode_state = get_phase(datetime.datetime.now())

    logger.log("Connected!")
    #while mode_state == 'cozmo' and cozmo.is_connected():
    while cozmo.is_connected():
        # Report updates to trackers
        focus_state = focus_tracker.get_state()

        # Start camera if approaching sedentary, snooze, or acivity limit
        # get the timer recording checks from the tracker, and no recording if baseline mode
        #if  not recorder.recording and focus_tracker.is_start_recording():
        if focus_tracker.is_start_recording():
            logger.log("Recording start.", printing=True)
            webcam_pub.publish(True)

        #status_logger.log("--- Logic Status Summary ---")
        #status_logger.log("Focus state is: {}".format(focus_state))
        #status_logger.log("Cozmo getting attention: {}".format(cozmo.getting_attention()))
        #status_logger.log("Cozmo is idle: {}".format(cozmo.is_idle()))
        #status_logger.log("Cozmo behavior is: {}".format(cozmo.current_behavior))
        #status_logger.log("Cozmo is picked up: {}".format(cozmo.is_picked_up()))
        #status_logger.log("Cozmo is charging: {}".format(cozmo.is_charging()))

        '''
        state transitions:
        stop cozmo:
            user stood up
            cozmo ignored
            button pressed
            cozmo picked up
        start cozmo
            time for break
            snooze time up
        change behavior
            goal routine??
            stand up routine??

        '''
        new_behavior = "idle"
        log_message = "Working"
        i=0

        # If we currently want cozmo...
        if focus_state is UserBreakState.NeedCozmo:
            # if cozmo is not currently active...
            if not focus_tracker.cozmo_active:
                # Start up cozmo
                focus_tracker.start_cozmo()
                if mode_state == "cozmo":
                    new_behavior = "attention"
                    if cozmo.is_picked_up():
                        new_behavior = "airborne"
                cozmo.run_new_behavior(new_behavior)
                log_message = "Cozmo Called."
            # if cozmo is running and picked up...
            elif not cozmo.is_idle() and cozmo.is_picked_up() and cozmo.getting_attention():
                if mode_state == "cozmo":
                    cozmo.run_new_behavior("airborne")
            # else if cozmo is currently idle and has probably completed the attention behavior...
            elif cozmo.is_idle():
                # Cozmo was ignored, call snooze
                focus_tracker.start_snooze()
                log_message = "Start snooze"
                cozmo.run_new_behavior(new_behavior)
        # else if we want to snooze...
        elif focus_state is UserBreakState.SnoozeTriggered:
            # call snooze
            focus_tracker.start_snooze()
            cozmo.run_new_behavior(new_behavior)
            log_message = "Start snooze"
        # else if we're starting a break
        elif focus_state is UserBreakState.BreakRequest:
            # start the break
            focus_tracker.break_started()
            log_message = "Break started"
            # if cozmo is currently active...
            if not cozmo.is_idle():
                # and it's the attention getting phase
                if cozmo.getting_attention() or cozmo.current_behavior == "airborne":
                    # run a success routing
                    if mode_state == "cozmo":
                        cozmo.run_new_behavior("goal")
                    log_message = "Goal achieved start."
                # or it's not the success routine
                elif not cozmo.current_behavior == "goal":
                    # stop cozmo
                    cozmo.run_new_behavior(new_behavior)
        # else if there's no break
        elif focus_state is UserBreakState.NoBreak:
            # if cozmo is currently active...
            if not cozmo.is_idle():
                # and it's the attention or airborne states
                if cozmo.current_behavior == "attention" or cozmo.current_behavior == "airborne":
                    # then stop and cancel the likely break request
                    focus_tracker.cancel_break()
                    cozmo.run_new_behavior(new_behavior)
                    log_message = "Attention Cancel"
        # else if we're on a break...
        elif focus_state is UserBreakState.OnBreak:
            log_message = "On break"
        # else if the break has ended...
        elif focus_state is UserBreakState.BreakEnded:
            log_message = "Break ended"
        

        # A routine is going, continue it
        cozmo.tick()

        # Update our mode_state
        mode_state = get_phase(datetime.datetime.now())

        # If there is a new log message, log and print
        if log_message != log_message_old:
            log_message_old = log_message
            logger.log(log_message, printing=True)

        if time.time() - last_state_save > save_frequency:
            save_state()
            last_state_save = time.time()

        r.sleep()

    #current_routine.stop()
    logger.log('STOP', printing=True)


def get_phase(current_date):
    elapsed_days = (current_date - start_date)
    for i in range(len(phase_list)):
        if elapsed_days > phase_times[i]:
            elapsed_days = elapsed_days - phase_times[i]
        else:
            return phase_list[i]
    return 'end'

def save_state():
    focus_tracker.save_state()
    config = configparser.ConfigParser()
    config.read(str(configPath))
    if not config.has_section('SaveState'):
        config.add_section('SaveState')
    config.set('SaveState','StartDate', str(start_date))
    with open(str(configPath), 'w') as configfile:
            config.write(configfile)

# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Make sure subprocesses are terminated
    #if recording_process is not None and recording_process.poll() is None:
    #    os.killpg(os.getpgid(recording_process.pid), signal.SIGTERM)
    # Log, then shut down remaining
    logger.log('STOP', printing=True)
    if cozmo is not None:
        cozmo.disconnect()
    save_state()
    rospy.signal_shutdown("User requested shutdown.")
    sys.exit(0)


if __name__ == '__main__':
    # Allow keyboard exit
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    # Start ROS node for trackers
    rospy.init_node('cozmo', anonymous=True)
    webcam_pub = rospy.Publisher("/usb/webcam", Bool, queue_size=1)
    r = rospy.Rate(10)

    logger = logger_minimal.Logger('main')
    status_logger = logger_minimal.Logger('status')

    logger.log("Checking for existing state..")
    #state_path = Path("~/.wcr_state_save.ini")
    #if state_path.is_file():
    #    logger.log("Restarting from saved state...")
    #    stateFile = configparser.ConfigParser()
    #    stateFile.read(str(state_path))
    #    start_date = stateFile['SaveState'][]


    logger.log("Parsing configuration...")
    configPath = pathlib.Path(__file__).resolve().parents[1] / 'config.ini'
    #if not configPath.is_file():
    config = configparser.ConfigParser()
    config.read(str(configPath))
    breaktime = config['SetupSettings'].getfloat('MaxBreakInterruptHr')
    breakwindow = config['SetupSettings'].getfloat('InterruptWindowHr')
    save_frequency = config['SetupSettings'].getfloat('SaveFrequency')
    # Convert to seconds...
    save_frequency = save_frequency*60
    phase_list = config['SetupSettings']['PhaseList']
    phase_list = phase_list.split(", ")
    phase_times_string = config['SetupSettings']['PhaseDurations'].split(", ")
    phase_times = [datetime.timedelta(days=float(ele)) for ele in phase_times_string]
    is_testing = config['SetupSettings'].getboolean('Testing')
    start_date = datetime.datetime.now()
    current_date = datetime.datetime.now()

    if config.has_section("SaveState"):
        logger.log("Found state info, restarting from last known state...")
        start_date = datetime.datetime.strptime(config['SaveState']['StartDate'],'%Y-%m-%d %H:%M:%S.%f')

    mode_state = get_phase(datetime.datetime.now())
    #if mode_state == 'end':
    #    logger.log("Study is complete")
    #    logger.log("STOP")

    cozmo = None

    # Start the trackers
    logger.log('Initializing trackers...')
    #calendar_tracker = CalendarTracker.CalendarTracker()
    #if not baseMode:
    #    airborne_tracker = AirborneTracker.AirborneTracker()
    focus_tracker = FocusTracker.FocusTracker(mode_state != 'cozmo',testing = is_testing)

    logger.log("Starting system...")

    while not rospy.is_shutdown():
        try:
            mode_state = get_phase(datetime.datetime.now())
            if mode_state == 'cozmo':
                # Let's initialize this here, so it will reinitialize on exception...
                focus_tracker.change_mode(False)
            else:
                focus_tracker.change_mode(True)
            cozmo = cozmo_controller.CozmoController(testing=is_testing, log_level = logging.WARNING,
        protocol_log_level = logging.WARNING, robot_log_level = logging.WARNING)
            cozmo_loop(testing=is_testing)

        except Exception as e:
            logger.log("System run failed with error:\n {0}".format(e))
            logger.log(repr(e))
            logger.log(traceback.format_exc())

    logger.log('STOP', printing=True)
