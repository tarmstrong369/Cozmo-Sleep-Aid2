#!/usr/bin/env python3

import logger_minimal
import os
import pathlib
import sys
import signal
import pexpect
import rospy
from std_msgs.msg import Bool
import configparser

# Define device information: address should be identified and changed for each device
#with open(os.path.dirname(os.path.abspath(__file__))[0:-3] + 'id.chair_bluetooth', 'r') as f:
#    DEVICE_ADDRESS = f.readline().strip()
config = configparser.ConfigParser()
config.read(str(pathlib.Path(__file__).parents[1]) + '/config.ini')
DEVICE_ADDRESS = config['SetupSettings']['ChairSensorID']
TX_HANDLE = '0x0023'
RX_HANDLE = '0x0027'



# BLE messages are a byte array with spaced hexadecimal values that represent characters
#   - Not sure if this is always true since BLE devices can transmit way more data than what we're doing, but it works

# Transforms a byte array with hexademical numbers into a string
def hex_bytearray_to_string(incoming_message):
    return bytearray.fromhex(incoming_message.decode('utf-8')).decode()


# Transforms an outgoing string to a hexademical byte array - UNTESTED
def string_to_hex_values(outgoing_message):
    hex_vals = ''
    for c in outgoing_message:
        hex_vals = hex_vals + c.encode().hex() + ' '
    return hex_vals


# Requests the most recent message from the BLE device via gattool
def poll_ble():
    # Sends a request to read from the BLE's TX channel
    process.sendline("char-read-hnd " + TX_HANDLE)
    try:
        # Match for the expected response
        process.expect("Characteristic value/descriptor: ", timeout=3)
        # If the match did not timeout, extract the message
        process.expect("\r\n", timeout=3)
        return hex_bytearray_to_string(process.before)
    except pexpect.exceptions.TIMEOUT:
        # If anything timed out, return None
        return None


# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Terminate child process if it exists
    if process is not None and process.isalive():
        process.terminate(force=True)
    # Log, then shut down remaining
    logger.log('STOP')
    rospy.signal_shutdown("User requested shutdown.")
    sys.exit(0)


# ble_poller should be run as its own node
if __name__ == '__main__':
    # Setup callbacks, ROS and logging
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    rospy.init_node('chair', anonymous=True)
    pub = rospy.Publisher("/bluetooth/chair", Bool, queue_size=1)
    r = rospy.Rate(10)

    logger = logger_minimal.Logger('chair')
    logger.log('START', True)

    # State variables:
    process = None
    log_message_old = ""
    log_message = ""
    was_connected = False
    attempts = 0

    while not rospy.is_shutdown():
        # Repeatedly attempt to establish process
        if process is None:
            # Spawn a gatttool process for use later
            logger.log("Connecting...", True)
            process = pexpect.spawn("gatttool -t random -b {0} -I".format(DEVICE_ADDRESS))
            # Check if process connection status
            try:
                process.sendline("connect")
                process.expect("Connection successful", timeout=3)
                # If it does not timeout, it has succeeded
                #print("Connected!")
                logger.log('CONNECT', True)
                was_connected = True
            except pexpect.TIMEOUT:
                # Otherwise, delete the process and retry
                #print("Connected timed out.")
                process.terminate(force=True)
                process = None
                logger.log('TIMEOUT', True)
                # Trying to handle weird disconnect can't reconnect problem
                if was_connected:
                    if attempts >= 10:
                        os.system("shutdown /r /t 1")
                    else:
                        attempts += 1
                # Publish false as a default
                pub.publish(False)

        # The connection has been established, extract data
        else:
            message = poll_ble()
            if message is None:
                # None messages indicate a timeout or disconnect, delete the process
                #print('Disconnected... will try to reconnect.')
                process.terminate(force=True)
                process = None
                logger.log('DISCONNECT', True)
            else:
                # Extract the first character from the message for logging and publishing
                reading = int(message.split()[1])
                if reading < 100:
                    reading = 0
                    pub.publish(False)
                else:
                    reading = 1
                    pub.publish(True)

                pub.publish(bool(reading))

                log_message = str(reading)
                if log_message != log_message_old:
                    log_message_old = log_message
                    logger.log(log_message, True)

        r.sleep()
