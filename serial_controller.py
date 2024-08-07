#!/usr/bin/env python3
import sys
import logger_minimal
import os
import serial
import signal
import rospy
import time
from std_msgs.msg import Bool

# This code handles getting and passing data from the serial line, which includes 
#      - the keyboard activity sensor (vibration sensor)
#      - button response (snooze button)


# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Log, then shut down remaining
    logger.log('STOP')
    rospy.signal_shutdown("User requested shutdown.")
    sys.exit(0)

if __name__ == '__main__':
    # Setup callbacks, ROS and logging
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)
    rospy.init_node('taps', anonymous=True)
    key_pub = rospy.Publisher("/serial/keyboard", Bool, queue_size=100)
    btn_pub = rospy.Publisher("/serial/button", Bool, queue_size=1)
    r = rospy.Rate(1000)

    logger = logger_minimal.Logger('taps')
    logger.log('START', True)

    try:
        tap_sensor = serial.Serial('/dev/ttyACM0', timeout=None, baudrate=115200)
        logger.log('Connected', True)
        while not rospy.is_shutdown():
            msg = tap_sensor.readline().decode().strip()
            if msg == "key":
                logger.log('Key pressed', True)
                key_pub.publish(True)
            elif msg == "btn":
                logger.log('Button pressed', True)
                btn_pub.publish(True)
            r.sleep()
    except serial.SerialException as e:
        logger.log("Serial exception occurred: {}".format(e))
    except Exception as e:
        logger.log("Error occured at {}".format(e))
            
