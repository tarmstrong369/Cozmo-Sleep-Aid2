import time
import pycozmo
import numpy as np
from PIL import Image


# with pycozmo.connect() as cli:

#     # Set volume to ~75%.
#     cli.set_volume(50000)

#     # A 22 kHz, 16-bit, mono file is required.
#     cli.play_audio("hello.wav")
#     cli.wait_for(pycozmo.event.EvtAudioCompleted)





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
        j=3
        i=0
        while i<j:
            #Spin
            cli.drive_wheels(lwheel_speed=50.0, rwheel_speed=-50.0, duration=5.0)
            cli.set_lift_height(pycozmo.MAX_LIFT_HEIGHT.mm)
            time.sleep(1)
            cli.set_lift_height(pycozmo.MIN_LIFT_HEIGHT.mm)
            time.sleep(1)
            i=i+1



def bedtime1():
  
    with pycozmo.connect() as cli:
        #Roll Forward
        cli.drive_wheels(lwheel_speed=25.0, rwheel_speed=25.0, duration=4.0)

        #Spin length
        j=3
        i=0
        while i<j:
            #Spin
            cli.drive_wheels(lwheel_speed=25.0, rwheel_speed=-25.0, duration=12.0)
            cli.set_lift_height(pycozmo.MAX_LIFT_HEIGHT.mm)
            time.sleep(1)
            cli.set_lift_height(pycozmo.MIN_LIFT_HEIGHT.mm)
            time.sleep(1)
            i=i+1


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
        

# bedtime1()
