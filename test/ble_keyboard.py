#!/usr/bin/env python3

import sys
import signal
import rospy
from std_msgs.msg import Bool

import contextlib
with contextlib.redirect_stdout(None):
    import pygame
    from pygame.locals import *


# Callback to handle SIGINT and SIGTERM
def keyboard_interrupt_callback(_1, _2):
    rospy.signal_shutdown("User requested shutdown.")
    sys.exit(0)


# ble_poller_placeholder should be run as its own node
if __name__ == '__main__':
    # Setup callbacks, ROS, and pygame
    signal.signal(signal.SIGINT, keyboard_interrupt_callback)

    rospy.init_node('ble_tracker', anonymous=True)
    pub = rospy.Publisher("/arduino/chair_sensor", Bool, queue_size=1)
    r = rospy.Rate(10)

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('BLE Keyboard Control Panel')
    screen.fill((159, 182, 205))
    pygame.display.update()

    # State variables:
    sitting = True
    while not rospy.is_shutdown():
        # Refresh events and extract currently pressed keys
        pygame.event.pump()
        keys = pygame.key.get_pressed()

        # Determine if a change in the sitting status is needed
        if keys[K_UP]:
            sitting = False
        elif keys[K_DOWN]:
            sitting = True

        pub.publish(sitting)

        # Exit if correct keys pressed
        if keys[K_ESCAPE] or ((keys[K_LCTRL] or keys[K_RCTRL]) and keys[K_c]):
            break

        r.sleep()
