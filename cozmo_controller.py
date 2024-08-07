#!/usr/bin/env python3
import pycozmo
import time
import queue
import sys
import logger_minimal
import subprocess
import math
import logging
import pathlib
import os
from routines import actionlib
from routines import emotions
from custom_exceptions import IncorrectCozmoVersionError

def setup_basic_logging(
        log_level = None,
        protocol_log_level = None,
        robot_log_level = None,
        filename="pycozmo_") -> None:

    if log_level is None:
        log_level = os.environ.get('PYCOZMO_LOG_LEVEL', logging.INFO)
    if protocol_log_level is None:
        protocol_log_level = os.environ.get('PYCOZMO_PROTOCOL_LOG_LEVEL', logging.INFO)
    if robot_log_level is None:
        # Keeping the default to WARNING due to "AnimationController.IsReadyToPlay.BufferStarved" messages.
        robot_log_level = os.environ.get('PYCOZMO_ROBOT_LOG_LEVEL', logging.WARNING)
    filepath = str(pathlib.Path(__file__).parents[1]) + '/logs/logfiles/' + filename
    handler = logging.handlers.WatchedFileHandler(filepath+"general.log")
    protocol_handler = logging.handlers.WatchedFileHandler(filepath+"protocol.log")
    robot_handler = logging.handlers.WatchedFileHandler(filepath+"robot.log")
    reaction_handler = logging.handlers.WatchedFileHandler(filepath+"reaction.log")
    behavior_handler = logging.handlers.WatchedFileHandler(filepath+"behavior.log")
    animation_handler = logging.handlers.WatchedFileHandler(filepath+"animation.log")
    formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(name)-20s %(levelname)-8s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S")
    handler.setFormatter(formatter)
    protocol_handler.setFormatter(formatter)
    robot_handler.setFormatter(formatter)
    reaction_handler.setFormatter(formatter)
    behavior_handler.setFormatter(formatter)
    animation_handler.setFormatter(formatter)
    pycozmo.logger.addHandler(handler)
    pycozmo.logger.setLevel(log_level)
    pycozmo.logger_protocol.addHandler(protocol_handler)
    pycozmo.logger_protocol.setLevel(protocol_log_level)
    pycozmo.logger_robot.addHandler(robot_handler)
    pycozmo.logger_robot.setLevel(robot_log_level)
    pycozmo.logger_reaction.addHandler(reaction_handler)
    pycozmo.logger_reaction.setLevel(robot_log_level)
    pycozmo.logger_behavior.addHandler(behavior_handler)
    pycozmo.logger_behavior.setLevel(robot_log_level)
    pycozmo.logger_animation.addHandler(animation_handler)
    pycozmo.logger_animation.setLevel(robot_log_level)


class CozmoController:
    _behavior_queue = queue.Queue()
    waiting_until = 0
    current_behavior = "idle"
    old_behavior = "idle"
    is_starved = False
    _last_ping_time = time.time()
    _ping_timeout = 30

    def __init__(self, setup_logging = True, testing=False, log_level = logging.DEBUG,
            protocol_log_level = logging.DEBUG, robot_log_level = logging.DEBUG):
        self.cli = None
        self.logger = logger_minimal.Logger('robot')
        if setup_logging:
            setup_basic_logging(log_level=log_level, protocol_log_level=protocol_log_level, robot_log_level=robot_log_level)

    def connect(self, 
            protocol_log_messages = None,
            auto_initialize = True,
            enable_animations = True,
            enable_procedural_face = True):
        
        if self.cli is None:
            try:
                self.cli = pycozmo.client.Client(
                protocol_log_messages=protocol_log_messages,
                auto_initialize=auto_initialize,
                enable_animations=enable_animations,
                enable_procedural_face=enable_procedural_face)
                # Don't need debug data system reset anymore, just doing pings nao
                #self.cli.add_handler(pycozmo.protocol_encoder.DebugData, self._on_debug_data)
                self.cli.add_handler(pycozmo.protocol_encoder.Ping, self._on_ping)
                # TODO: add event handler to swap lights for charging/not charging, well... maybe
                #self.cli.add_handler(pycozmo.event.EvtRobotChargingChange, self.on_charging_change)
                self.cli.start()
                self.cli.connect()
                self.cli.wait_for_robot()
                if self.cli.robot_fw_sig["version"] != 2381:
                    raise IncorrectCozmoVersionError(self.cli.robot_fw_sig["version"])

                self.cli.load_anims()
                self.cli.set_lift_height(0)
                self.cli.set_head_angle(0)
                self.cli.set_volume(0.0)
                self.logger.log("Connected")
                #self.set_static()
                
                return 0
            except IncorrectCozmoVersionError as e:
                self.logger.log(e)
                self.cli.disconnect()
                self.cli.stop()
                self.cli = None
                update_code = subprocess.call(["pycozmo_update.py", "/home/cozmo/.pycozmo/assets/cozmo_resources/config/engine/firmware/cozmo.safe"])
                if update_code == 0:
                    self.logger.log("Updated firmware successfully!")
                    return 2
                else:
                    self.logger.log("Failed to update firmware. Will keep trying.")
                    return 3
                #proc = subprocess.Popen(['php', '-f', 'test.php'],
                #        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                #stdout,stderr = proc.communicate()
                #if proc.returncode != 0:
                #    raise Exception('Test error: ' + stderr)
                #return float(stdout)
            except pycozmo.exception.PyCozmoException as e:
                self.logger.log(e)
                self.cli.disconnect()
                self.cli.stop()
                self.cli = None
                return -1
                #sys.exit(1)
            except KeyboardInterrupt:
                self.logger.log("Interrupted...")
                self.cli.disconnect()
                self.cli.stop()
                self.cli = None
                return 1
                #sys.exit(0)
            except Exception as e:
                self.logger.log(e)
                self.cli.disconnect()
                self.cli.stop()
                self.cli = None
                return -1
        
    def disconnect(self):
        if self.cli is None:
            return
        self.logger.log("Disconnecting from Cozmo")
        self.cli.disconnect()
        self.cli.stop()
        self.cli = None
        #self.is_starved = False
        if not self._behavior_queue.empty():
            with self._behavior_queue.mutex:
                self._behavior_queue.queue.clear()
        self.waiting_until = 0
        self.current_behavior = "idle"
        self.old_behavior = "idle"

    def set_static(self):
        if not self.is_connected():
            return

        #self.logger.log("Setting static values")
        self.cli.set_lift_height(0)
        self.cli.set_head_angle(0)

        #self.logger.log("Setting backpack lights")
        self.logger.log("Battery value is: {}".format(self.cli.battery_voltage))
        if self.cli.is_charging or self.cli.battery_voltage >= 4:
            self.set_lights('green')
        elif self.cli.battery_voltage >= 3.85:
            self.set_lights('yellow')
        else:
            self.set_lights('red')

    def on_charging_change(self, state: bool):
        # TODO: This entire thing idk
        if not self.is_connected():
            return
        self.logger.log("Updating lights")
        # TODO: Lights not seeming to work??
        if not state:
            # If the battery is low (which is less than 3.7) flash red, 
            if(self.cli.battery_voltage <= 3.7):
                self.set_lights("red")
            # if less than half (3.85), flash yellow
            elif(self.cli.battery_voltage <= 3.85):
                self.set_lights("yellow")
            # else we're greeeen
            else:
                self.set_lights("green")
        # else flash green
        else:
            self.set_lights("green")

    def _on_debug_data(self, cli, pkt: pycozmo.protocol_encoder.DebugData):
        del cli
        #if pkt.name_id == 169:
            #self.logger.log("Buffer starved, connection closed, restart")
            #self.is_starved = True
        #msg = robot_debug.get_debug_message(pkt.name_id, pkt.format_id, pkt.args)
        #logger_robot.log(robot_debug.get_log_level(pkt.level), msg)

    def _on_ping(self, cli, pkt: pycozmo.protocol_encoder.Ping):
        del cli
        self._last_ping_time = time.time()
        #self.logger.log("Ping time: {}".format(self._last_ping_time))

    def set_lights(self, light_color="green"):
        self.logger.log("Setting lights")
        light = pycozmo.lights.off
        if light_color == "green":
            light = pycozmo.lights.green
        elif light_color == "yellow":
            light = pycozmo.lights.Color(name='yellow', int_color=0xffa400ff)
        elif light_color == "red":
            light = pycozmo.lights.red

        light_state = pycozmo.protocol_encoder.LightState(on_color=light.to_int16(), 
                                                        off_color=pycozmo.lights.off.to_int16(), 
                                                        on_frames = 20, 
                                                        off_frames=20, 
                                                        transition_on_frames=20, 
                                                        transition_off_frames=20)
        self.cli.set_center_backpack_lights(light_state)

    def run_new_behavior(self, behavior_name):
        if not self.is_connected():
            return
        
        if self.current_behavior == behavior_name:
            return
        self.logger.log("Starting new behavior: {}".format(behavior_name))
        self.stop_behavior()

        if behavior_name != "idle" and self.cli.is_on_charger:
            self.logger.log("Driving off charger")
            self.cli.drive_off_charger_contacts()

        behavior_list = self.get_behavior(behavior_name)
        self.logger.log("Start behavior with items:\n{0}".format(behavior_list))
        self.old_behavior = self.current_behavior
        self.current_behavior = behavior_name
        
        for behavior in behavior_list:
            self._behavior_queue.put(behavior)

    def stop_behavior(self):
        if not self.is_connected():
            return
        self.logger.log("Stop current behavior")
        # Tell the client to cancel any current animation
        self.cli.cancel_anim()
        # Clear the remaining queue
        if not self._behavior_queue.empty():
            with self._behavior_queue.mutex:
                self._behavior_queue.queue.clear()
        #self.behavior_name = "idle"
        self.set_static()

    def get_behavior(self, behavior_name):
        behavior_list = []

        if behavior_name == "attention":
            #actions = actionlib.library.get_act_type()
            #behavior_list, repeats = actionlib.library.get_attention_getting_walk_route(actions)
            behavior_list, group, behavior_names = actionlib.library.get_basic_attention_routine()
            actprint = "Behavior group is:\n"
            actprint += actionlib.ActionType(group).name
            actprint += "\n"
            actprint += "Behaviors are:\n"
            for act in behavior_names:
                actprint += act
                actprint += "\n"
            self.logger.log(actprint)

        elif behavior_name == "goal":
            behavior_list, repeats = actionlib.library.get_success_routine()

        elif behavior_name == "airborne":
            # behavior_list = [
            #     'cli.set_lift_height(0)',
            #     'cli.set_head_angle(0)',
            #     'cli.play_anim_group("CodeLabExcited")',
            #     'cli.play_anim_group("CodeLabHappy")',
            #     'cli.play_anim_group("CodeLabAmazed")',
            #     'cli.play_anim_group("CodeLabIdle")',
            # ]
            behavior_list = [
                #'cli.play_anim("anim_codelab_excited_static")',
                #'cli.play_anim("anim_codelab_happy_static")',
                #'cli.play_anim("anim_codelab_amazed_static")',
                #'cli.play_anim("anim_codelab_idle_static")',
                'cli.play_anim("anim_SHARELab_on_airborne")',
            ]

        elif behavior_name == "standing":
            behavior_list = [
                'cli.play_anim("anim_codelab_excited_static")',
                'cli.play_anim("anim_SHARELab_goal_achieved")',
                'cli.play_anim("anim_codelab_amazed_static")',
                'cli.play_anim("anim_codelab_idle_static")',
            ]
        elif behavior_name == "charging":
            behavior_list = [
                'cli.drive_straight(-200, 300)',
            ]
        elif "emotions" in behavior_name:
            behavior_list = eval(behavior_name + "()")


        return behavior_list

    def wait(self, duration):
        # duration in seconds
        self.waiting_until = time.time() + duration

    def tick(self):
        if not self.is_connected():
            return
        if self.is_idle():
            if self.current_behavior != "idle":
                self.logger.log("Reset to base state")
                self.current_behavior = "idle"
                self.set_static()
            # if we're idle and we're on the charger but not charging...
            elif self.cli.is_on_charger and not self.cli.is_charging:
                if self.cli.battery_voltage < 4:
                    self.run_new_behavior("charging")
        elif not self.is_waiting() and not self._behavior_queue.empty():
            if not self.cli.anim_controller.playing_animation and not self.cli.robot_moving:
                if self.cli.is_on_charger:
                    self.cli.drive_off_charger_contacts()
                else:
                    current_action = self._behavior_queue.get()
                    self.logger.log("Starting action: {}".format(current_action))
                    eval('self.' + current_action)

    #@property
    def is_waiting(self):
        return self.waiting_until > time.time()

    def is_idle(self):
        return self._behavior_queue.empty() and not self.cli.anim_controller.playing_animation and not self.cli.robot_moving and not self.is_waiting()

    def is_charging(self):
        return self.cli.is_charging

    def is_picked_up(self):
        return self.cli.robot_picked_up

    def getting_attention(self):
        return self.current_behavior == "attention"

    def was_getting_attention(self):
        return self.old_behavior == "attention"

    def is_connected(self):
        if self.cli != None:
            #if self.is_starved:
            #    self.disconnect()
            #    return False
            if time.time() - self._last_ping_time >  self._ping_timeout:
                self.disconnect()
                return False
            else:
                return True

        else:
            return False



