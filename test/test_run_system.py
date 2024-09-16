#!/usr/bin/env python3

import sys
import logger_minimal
import os
import signal
import rospy
import time
from test import test_cases
from std_msgs.msg import Bool

# TODO: I don't know the state of this...

# This code provides keyboard, button, and chair bluetooth generated data to
# test the functionality of the system including edge cases, failure cases, and all 
# possible logic permutations. The list of possible cases is shown in test_cases.py, and the code
# runs every sequence permutation of this list.

# To run: Set mode to testing in the config file


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
    rospy.init_node('sim', anonymous=True)
    key_pub = rospy.Publisher("/serial/keyboard", Bool, queue_size=1)
    btn_pub = rospy.Publisher("/serial/button", Bool, queue_size=1)
    chair_pub = rospy.Publisher("/bluetooth/chair", Bool, queue_size=1)
    r = rospy.Rate(10)

    logger = logger_minimal.Logger('sim')
    logger.log('START', True)

    all_permutation_sets = list(itertools.permutations(CASE_LIST))
    
    while not rospy.is_shutdown():
        #for permutation_set in all_permutation_sets:
        #    for case in permutation_set:
        #        eval('test_cases.' + case + '()')
            #key_pub.publish(True)
            #btn_pub.publish(True)
        choice = random.randint(1,100)
        if choice < 30:
            logger.log('Key pressed', True)
            key_pub.publish(True)
        elif choice < 35:
            logger.log('Button pressed', True)
            btn_pub.publish(True)
        elif choice < 36:
            logger.log("user stand up", True)
            chair_pub.publish(False)
        else:
            chair_pub.publish(True)

        r.sleep()
