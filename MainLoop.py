from datetime import datetime
import Cozmosleeptest as cozmo
import time
import queue
import sys
# import logger_minimal
import subprocess
import math
import logging
import pathlib
import os
import pycozmo

#Wake up time (24hr):
wakeuphr=15
wakeupmin=22

#Bed time (24hr):
bedtimehr=14
bedtimemin=22


def istime(hr,min,targhr,targmin,margin=1):
    current_time = datetime.now()
    target_time = datetime(current_time.year, current_time.month, current_time.day, targhr, targmin)
    return abs((current_time - target_time).total_seconds()) < margin



while True:
    current_time = datetime.now()
    hour = current_time.hour
    minute = current_time.minute
    second = current_time.second
    # Cozmoconnect(pycozmo)
    # print(second)
    if istime(hour, minute, wakeuphr, wakeupmin):
        print('Wake Up!')
        cozmo.wakeup1()
        time.sleep(10)
        # timeleft=60-current_time.second
        # time.sleep(timeleft)  # Sleep for 60 seconds to avoid multiple triggers

    elif istime(hour, minute, bedtimehr, bedtimemin):
        print('Bedtime!')
        cozmo.bedtime1()
        time.sleep(10)
        # timeleft=60-current_time.second
        # time.sleep(timeleft)  # Sleep for 60 seconds to avoid multiple triggers
        

    else:
        print('nope')
        # timeleft=60-current_time.second
        # time.sleep(timeleft)
    
    time.sleep(1)

