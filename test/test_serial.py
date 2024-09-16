#!/usr/bin/env python3
import sys
import os
import serial
import signal
import time
from std_msgs.msg import Bool

# This code handles getting and passing data from the serial line, which includes 
#      - the keyboard activity sensor (vibration sensor)
#      - button response (snooze button)


# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Log, then shut down remaining
    print('STOP')
    sys.exit(0)

if __name__ == '__main__':
    # Setup callbacks, ROS and logging
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    try:
        tap_sensor = serial.Serial('/dev/ttyACM0', timeout=None, baudrate=115200)
        print('Connected', flush=True)
        while True:
            msg = tap_sensor.readline().decode().strip()
            if msg == "key":
                print('Key pressed', flush=True)
            elif msg == "btn":
                print('Button pressed', flush=True)
            time.sleep(.5)
    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e), flush = True)
    except Exception as e:
        print("Error occured at {}".format(e), flush = True)
