import time
import pycozmo
import numpy as np
import serial
import time
import queue
import sys
# import logger_minimal
import subprocess
import math
import logging
import pathlib
import os

from PIL import Image
from threading import Timer



def Cozmoconnect(self,
            protocol_log_messages = None,
            auto_initialize = True,
            enable_animations = True,
            enable_procedural_face = True):
        
        
        with pycozmo.connect() as cli:

        
            if cli is None:
                try:
                    cli = pycozmo.client.Client(
                    protocol_log_messages=protocol_log_messages,
                    auto_initialize=auto_initialize,
                    enable_animations=enable_animations,
                    enable_procedural_face=enable_procedural_face)
                    # Don't need debug data system reset anymore, just doing pings nao
                    #self.cli.add_handler(pycozmo.protocol_encoder.DebugData, self._on_debug_data)
                    cli.add_handler(pycozmo.protocol_encoder.Ping, self._on_ping)
                    # TODO: add event handler to swap lights for charging/not charging, well... maybe
                    #self.cli.add_handler(pycozmo.event.EvtRobotChargingChange, self.on_charging_change)
                    cli.start()
                    cli.connect()
                    cli.wait_for_robot()
                    if cli.robot_fw_sig["version"] != 2381:
                        raise IncorrectCozmoVersionError(cli.robot_fw_sig["version"])

                    cli.load_anims()
                    cli.set_lift_height(0)
                    cli.set_head_angle(0)
                    cli.set_volume(0.0)
                    # self.logger.log("Connected")
                    print("Connected!")
                    #self.set_static()
                    
                    return 0
                except IncorrectCozmoVersionError as e:
                    self.logger.log(e)
                    cli.disconnect()
                    cli.stop()
                    cli = None
                    update_code = subprocess.call(["pycozmo_update.py", "/home/cozmo/.pycozmo/assets/cozmo_resources/config/engine/firmware/cozmo.safe"])
                    if update_code == 0:
                        # self.logger.log("Updated firmware successfully!")
                        print("Updated firmware successfully!")
                        return 2
                    else:
                        # self.logger.log("Failed to update firmware. Will keep trying.")
                        print("Failed to update firmware. Will keep trying.")
                        return 3
                    #proc = subprocess.Popen(['php', '-f', 'test.php'],
                    #        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    #stdout,stderr = proc.communicate()
                    #if proc.returncode != 0:
                    #    raise Exception('Test error: ' + stderr)
                    #return float(stdout)
                except pycozmo.exception.PyCozmoException as e:
                    self.logger.log(e)
                    cli.disconnect()
                    cli.stop()
                    cli = None
                    return -1
                    #sys.exit(1)
                except KeyboardInterrupt:
                    # self.logger.log("Interrupted...")
                    print("Interrupted...")
                    cli.disconnect()
                    cli.stop()
                    cli = None
                    return 1
                    #sys.exit(0)
                except Exception as e:
                    self.logger.log(e)
                    cli.disconnect()
                    cli.stop()
                    cli = None
                    return -1
            






def wakeup1():
  
    with pycozmo.connect() as cli:
             # Raise head.
        angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
        cli.set_head_angle(angle)
        time.sleep(1)

        # List of face expressions.
        expressions = [
            # pycozmo.expressions.Anger(),
            # pycozmo.expressions.Sadness(),
            # pycozmo.expressions.Happiness(),
            # pycozmo.expressions.Disgust(),
            # pycozmo.expressions.Fear(),
            # pycozmo.expressions.Pleading(),
            # pycozmo.expressions.Vulnerability(),
            # pycozmo.expressions.Despair(),
            # pycozmo.expressions.Guilt(),
            # pycozmo.expressions.Disappointment(),
            # pycozmo.expressions.Embarrassment(),
            # pycozmo.expressions.Horror(),
            # pycozmo.expressions.Skepticism(),
            # pycozmo.expressions.Annoyance(),
            # pycozmo.expressions.Fury(),
            # pycozmo.expressions.Suspicion(),
            # pycozmo.expressions.Rejection(),
            # pycozmo.expressions.Boredom(),
            # pycozmo.expressions.Tiredness(),
            pycozmo.expressions.Asleep(),
            pycozmo.expressions.Surprise(),
            # pycozmo.expressions.Confusion(),
            # pycozmo.expressions.Amazement(),
            pycozmo.expressions.Excitement(),
        ]

        # Base face expression.
        base_face = pycozmo.expressions.Neutral()

        rate = pycozmo.robot.FRAME_RATE
        timer = pycozmo.util.FPSTimer(rate)
        for expression in expressions:

            # Transition from base face to expression and back.
            for from_face, to_face in ((base_face, expression), (expression, base_face)):

                if to_face != base_face:
                    print(to_face.__class__.__name__)

                # Generate transition frames.
                face_generator = pycozmo.procedural_face.interpolate(from_face, to_face, rate)
                for face in face_generator:

                    # Render face image.
                    im = face.render()

                    # The Cozmo protocol expects a 128x32 image, so take only the even lines.
                    np_im = np.array(im)
                    np_im2 = np_im[::2]
                    im2 = Image.fromarray(np_im2)

                    # Display face image.
                    cli.display_image(im2)

                    # Maintain frame rate.
                    timer.sleep()

                # Pause for 1s.
                for i in range(rate):
                    timer.sleep()
        #Roll Forward
        cli.drive_wheels(lwheel_speed=50.0, rwheel_speed=50.0, duration=3.0)

        #Spin length
        j = 3
        i = 0
        # how long to turn 
        turning = 0
        while i<j:
            # SNOOZE feature 
            if button():  # When clicked the buttons input is b'btn\r\n' (this is commend for buttons maybe...)
                print("snooze activated")
                cli.drive_wheels(lwheel_speed=-50.0, rwheel_speed=50.0, duration=turning)  # Turn to so back of robot faces charger (i.e., turn the robot 180)
                cli.drive_wheels(lwheel_speed=-50.0, rwheel_speed=-50.0, duration=3.0)  # roll robot back to base 
                # Timer(7 * 60, wakeup1).start() # Timer is based of seconds (7 *60 =  7 minutes) wait 7 minutes then run wakeup1 again
                # Timer(10, bedtime1).start()  # Timer is based of seconds (7 *60 =  7 minutes) wait 7 minutes then run wakeup1 again
                Cozmoconnect(pycozmo)
            else:
            #Spin
                cli.drive_wheels(lwheel_speed=50.0, rwheel_speed=-50.0, duration=7.0)
                cli.set_lift_height(pycozmo.MAX_LIFT_HEIGHT.mm)
                time.sleep(1)
                cli.set_lift_height(pycozmo.MIN_LIFT_HEIGHT.mm)
                time.sleep(1)
                i = i + 1
                turning = turning + 5.0  # saves the amount of times turn and resets the robot when snooze is press
 


def bedtime1():
  
    with pycozmo.connect() as cli:
        #Roll Forward
        cli.drive_wheels(lwheel_speed=25.0, rwheel_speed=25.0, duration=4.0)

        #Spin length
        j = 3
        i = 0
        # how long to turn when snooze is activated when snooze is activated
        turning = 0
        while i<j:
            # SNOOZE feature 
            if button():  # When clicked the buttons input is b'btn\r\n' (this is commend for buttons maybe...)
                print("snooze activated")
                cli.drive_wheels(lwheel_speed=-25.0, rwheel_speed=25.0, duration=turning)  # Turn to so back of robot faces charger (i.e., turn the robot 180)
                cli.drive_wheels(lwheel_speed=-25.0, rwheel_speed=-25.0, duration=4.0)  # roll robot back to base 
                # Timer(7 * 60, bedtime1).start()  # Timer is based of seconds (7 *60 =  7 minutes) wait 7 minutes then run wakeup1 again
                Timer(10, bedtime1).start()  # Timer is based of seconds (7 *60 =  7 minutes) wait 7 minutes then run wakeup1 again
                Cozmoconnect(pycozmo)
            else:
                #Spin
                cli.drive_wheels(lwheel_speed=25.0, rwheel_speed=-25.0, duration=12.0)
                cli.set_lift_height(pycozmo.MAX_LIFT_HEIGHT.mm)
                time.sleep(1)
                cli.set_lift_height(pycozmo.MIN_LIFT_HEIGHT.mm)
                time.sleep(1)
                i= i + 1
                turning = turning + 12.0  # saves the amount of times turn and resets the robot when snooze is press
             # Raise head.
        angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) / 2.0
        cli.set_head_angle(angle)
        time.sleep(1)

        # List of face expressions.
        expressions = [
            # pycozmo.expressions.Anger(),
            # pycozmo.expressions.Sadness(),
            # pycozmo.expressions.Happiness(),
            # pycozmo.expressions.Disgust(),
            # pycozmo.expressions.Fear(),
            pycozmo.expressions.Pleading(),
            # pycozmo.expressions.Vulnerability(),
            # pycozmo.expressions.Despair(),
            # pycozmo.expressions.Guilt(),
            # pycozmo.expressions.Disappointment(),
            # pycozmo.expressions.Embarrassment(),
            # pycozmo.expressions.Horror(),
            # pycozmo.expressions.Skepticism(),
            # pycozmo.expressions.Annoyance(),
            # pycozmo.expressions.Fury(),
            # pycozmo.expressions.Suspicion(),
            # pycozmo.expressions.Rejection(),
            # pycozmo.expressions.Boredom(),
            pycozmo.expressions.Tiredness(),
            pycozmo.expressions.Asleep(),
            pycozmo.expressions.Asleep(),
            # pycozmo.expressions.Surprise(),
            # pycozmo.expressions.Confusion(),
            # pycozmo.expressions.Amazement(),
            # pycozmo.expressions.Excitement(),
        ]

        # Base face expression.
        base_face = pycozmo.expressions.Neutral()

        rate = pycozmo.robot.FRAME_RATE
        timer = pycozmo.util.FPSTimer(rate)
        for expression in expressions:

            # Transition from base face to expression and back.
            for from_face, to_face in ((base_face, expression), (expression, base_face)):

                if to_face != base_face:
                    print(to_face.__class__.__name__)

                # Generate transition frames.
                face_generator = pycozmo.procedural_face.interpolate(from_face, to_face, rate)
                for face in face_generator:

                    # Render face image.
                    im = face.render()

                    # The Cozmo protocol expects a 128x32 image, so take only the even lines.
                    np_im = np.array(im)
                    np_im2 = np_im[::2]
                    im2 = Image.fromarray(np_im2)

                    # Display face image.
                    cli.display_image(im2)

                    # Maintain frame rate.
                    timer.sleep()

                # Pause for 1s.
                for i in range(rate):
                    timer.sleep()
        

def button():
    ser = serial.Serial("COM3", 9600)
    x = ser.readline()  # Reads the serial input 
    if x == b'btn\r\n':  # When clicked the buttons input is b'btn\r\n' (this is commend for buttons maybe...)
        print("button works")
        return True
    else:
        return False

