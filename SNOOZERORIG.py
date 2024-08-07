import time
import pycozmo
import numpy as np
import serial


from PIL import Image
from threading import Timer


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
            print(i)
            if button():  # When clicked the buttons input is b'btn\r\n' (this is commend for buttons maybe...)
                print("snooze activated")
                cli.drive_wheels(lwheel_speed=-50.0, rwheel_speed=50.0, duration=turning)  # Turn to so back of robot faces charger (i.e., turn the robot 180)
                cli.drive_wheels(lwheel_speed=-50.0, rwheel_speed=-50.0, duration=3.0)  # roll robot back to base 
                Timer(7 * 60, wakeup1).start() # Timer is based of seconds (7 *60 =  7 minutes) wait 7 minutes then run wakeup1 again
            else:
            #Spin
                cli.drive_wheels(lwheel_speed=50.0, rwheel_speed=-50.0, duration=5.0)
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
                Timer(7 * 60, bedtime1).start()  # Timer is based of seconds (7 *60 =  7 minutes) wait 7 minutes then run wakeup1 again
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

