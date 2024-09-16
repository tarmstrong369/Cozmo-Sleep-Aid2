#!/usr/bin/env python3
import os
import sys
import pathlib
print(str((pathlib.Path(__file__).resolve().parents[1] / "src")))
sys.path.append(str((pathlib.Path(__file__).resolve().parents[1] / "src")))
import cozmo_controller
import logger_minimal
from routines import actionlib
import configparser
import time
import traceback
import signal
import pycozmo

# Currently works for both testing lights and testing routines!

config = configparser.ConfigParser()
config.read(str(pathlib.Path(__file__).parents[0] / "test_config.ini"))

OPTIONS = []
if 'TestSelection' in config:
    for key in config['TestSelection']:
        if config['TestSelection'].getboolean(key):
            OPTIONS.append(str(key))

#OPTIONS = [
#    "lights",
#    "routines",
#    "behavior swap",
#]

#lights=[
#    ('green', pycozmo.lights.green.to_int16()),
#    ('yellow', pycozmo.lights.Color(int_color=0xffa400ff).to_int16()),
#    ('red', pycozmo.lights.red.to_int16())
#]
#lights=[
#    'green',
#    'yellow',
#    'red'
#]
#green = pycozmo.lights.green.to_int16()
#red = pycozmo.lights.red.to_int16()
#yellow = pycozmo.lights.Color(int_color=0xffa400ff).to_int16()
#off = pycozmo.lights.off.to_int16()
#transition_dur = 30
#hold_dur = 20

#lights = [
#    pycozmo.protocol_encoder.LightState(on_color=green, off_color=off, on_frames = hold_dur, off_frames=hold_dur, transition_on_frames=transition_dur, transition_off_frames=transition_dur),
#]

#behavior_list = [
#    'charging',
#    'airborne',
#    'standing',
#    'goal'
#]
#behavior_list = behavior_list + actionlib.library.list_p
#behavior_list = behavior_list + actionlib.library.list_unp
#behavior_list = behavior_list + actionlib.library.list_act
#behavior_list = behavior_list + actionlib.library.list_inact
#behavior_list = behavior_list + actionlib.library.list_act_p
#behavior_list = behavior_list + actionlib.library.list_act_unp
#behavior_list = behavior_list + actionlib.library.list_inact_p
#behavior_list = behavior_list + actionlib.library.list_inact_unp

def test_controller():
    # Try to connect to Cozmo...
    connect_code = cozmo.connect()
    # If connection failed...
    if connect_code != 0:
        if connect_code == -1:
            # Exception occurred, try reconnect
            #logger.log("Connection Failed")
            return 1
        elif connect_code == 1:
            # Keyboard interrupt, raise system quit
            shutdown_callback(1,1)
            return 2
        elif connect_code == 2:
            # Firmware updated successfully, try reconnect
            return 3
        elif connect_code ==3:
            # firmware update failed, try again here then try reconnect
            subprocess.call(["pycozmo_update.py", "/home/cozmo/.pycozmo/assets/cozmo_resources/config/engine/firmware/cozmo.safe"])
            return 4
    
    for option in OPTIONS:
        exit_code = 0
        if option == "lights":
            exit_code = test_lights()
        elif option == "routines":
            exit_code = test_routine_set()
        elif option == "behavior_swap":
            exit_code = test_behavior_swap()

        if exit_code != 0:
            return exit_code
    
    return exit_code

def test_lights():
    # test charging lights...
    logger.log("Testing backpack lights..")
    if config.has_section("LightColors"):
        for light in config['LightColors']:
            if config['LightColors'].getboolean(light):
                logger.log("Testing light: {}".format(light))
        #light_state = pycozmo.protocol_encoder.LightState(on_color=light[1], 
        #                                                off_color=pycozmo.lights.off.to_int16(), 
        #                                                on_frames = hold_dur, 
        #                                                off_frames=hold_dur, 
        #                                                transition_on_frames=transition_dur, 
        #                                                transition_off_frames=transition_dur)
                cozmo.set_lights(str(light))
        #cozmo.run_new_behavior('attention')
        #while not cozmo.is_idle():
        #    cozmo.tick()
                time.sleep(10)
        #cozmo.run_new_behavior('idle')
        #time.sleep(10)
    else:
        logger.log("No light colors were provided in config.")
        return 3
    #logger.log("Charging pattern...")
    #cozmo.set_lights(True)
    #time.sleep(10)
    #logger.log("Off charger pattern...")
    #logger.log("Low power pattern...")
    logger.log("Light test completed OK!")
    return 0


def test_routine_set():
    logger.log("Testing routine sets...")
    behavior_list = []
    if config.has_section("RoutinesList"):
        for behavior in config['RoutinesList']:
            if config['RoutinesList'].getboolean(behavior):
                behavior_list.append(str(behavior))
    if config.has_section("RoutineGroupsList"):
        for group in config['RoutineGroupsList']:
            if config['RoutineGroupsList'].getboolean(group):
                behavior_list = behavior_list + eval("actionlib.library." + str(group))

    if behavior_list == []:
        logger.log("No routines list was provided for testing.")
        return 4

    for current_routine in behavior_list:
        logger.log("Testing {} routine...".format(current_routine))
        cozmo.run_new_behavior(current_routine)
        while not cozmo.is_idle() and not cozmo.current_behavior == "idle":
            if cozmo.is_connected():
                cozmo.tick()
                time.sleep(1)
            else:
                logger.log("Failed on {} behavior".format(current_routine))
                return 1
        time.sleep(5)

    logger.log("Routine test completed OK!")
    return 0

def test_behavior_swap():
    logger.log("Testing behavior swapping...")
    if config.has_section("BehaviorTest"):
        pattern_set = config['BehaviorTest']['Pattern'].split(", ")
        pattern_times = [float(ele) for ele in config['BehaviorTest']['SwapTimeSec'].split(", ")]
    else:
        logger.log("No Behavior pattern was provided.")
        return 5
    for idx in range(len(pattern_set)):
        logger.log("Swapping to {} behavior...".format(pattern_set[idx]))
        cozmo.run_new_behavior(pattern_set[idx])
        start_time = time.time()
        waiting = True
        while waiting:
            if cozmo.is_connected():
                if time.time() - start_time > pattern_times[idx]:
                    waiting = False
                    logger.log("Swapping due to timer")
                elif pattern_set[idx] == "idle" or \
                    (not cozmo.is_idle() and not cozmo.current_behavior == "idle"):
                    cozmo.tick()
                    time.sleep(1)
                else:
                    waiting = False
            else:
                logger.log("Failed on {} behavior".format(pattern_set[idx]))
                return 2
    logger.log("Behavior swapping completed OK!")
    return 0

# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    print("User requested shutdown.")
    if cozmo is not None:
        if cozmo.is_connected():
            cozmo.disconnect()
    sys.exit(0)


if __name__ == '__main__':
    # Define global wrapper for sitting status
    #signal.signal(signal.SIGINT, keyboard_interrupt_callback)
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    logger = logger_minimal.Logger('testing')

    cozmo = None

    while True:
        try:
            logger.log("Starting Test")
            cozmo = cozmo_controller.CozmoController(setup_logging = False)
            exit_code = test_controller()
            if exit_code == 0:
                logger.log("Completed routine testing!")
            elif exit_code == 3:
                logger.log("No light colors were tested. Check config file!")
            elif exit_code == 1:
                logger.log("Failed while running routines!")
            elif exit_code == 4:
                logger.log("No routines were tested. Check config file!")
            elif exit_code == 2:
                logger.log("Failed while running behavior swap!")
            elif exit_code == 5:
                logger.log("No behavior swap tested. Check config file!")

            shutdown_callback(1,1)
        except Exception as e:
            logger.log("System run failed with error:\n {0}".format(e))
            logger.log(repr(e))
            logger.log(traceback.format_exc())
