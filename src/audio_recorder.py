#!/usr/bin/env python3

# From: https://www.codespeedy.com/save-webcam-video-in-python-using-opencv/
# and: https://learnopencv.com/read-write-and-display-a-video-using-opencv-cpp-python/
# and: https://stackoverflow.com/a/37209305

import cv2
import os
import pathlib
import sys
import datetime
import time
import configparser
import logger_minimal
import threading
import pyaudio
import wave
import signal
import rospy
import time
import subprocess
from std_msgs.msg import Bool




class AudioRecorder:
    recording = False
    
    def __init__(self, testing=False):
        config = configparser.ConfigParser()
        config.read(str(pathlib.Path(__file__).parents[1]) + '/config.ini')
        ID = config['SetupSettings']['SubjectID']
        self.audio_device = config['SetupSettings']['WebcamID']
        self.record_time = config['SetupSettings'].getfloat('RecordTime')
        # Convert to seconds
        self.record_time = self.record_time*60

        self.logger = logger_minimal.Logger('audio')
        self.logger.log("START")

        # Path format: identifier_YYYY-MM-DD_HH:MM:SS.log
        self.PATHFORMAT = ID + '_{0:04d}-{1:02d}-{2:02d}-{3:02d}:{4:02d}:{5:02d}'

        self.AUDIO_COMMAND = "ffmpeg -f alsa -i plughw:CARD={0},DEV=0 -strict -2 {1}.aac"

        self.folder_path = str(pathlib.Path(__file__).parents[1]) + '/logs/videos/'
        self.process_audio = None


    def start_record(self):
        audio_thread = threading.Thread(target=self.record)
        audio_thread.start()

    def record(self):
        self.recording = True
        self.logger.log("Start recording...")

        dt = datetime.datetime.now()
        filename = self.folder_path + self.PATHFORMAT.format(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)
        #filename = "test"

        self.process_audio = subprocess.Popen(self.AUDIO_COMMAND.format(self.audio_device, filename), shell=True, preexec_fn=os.setsid)


        start_time = time.time()
        current_time = time.time()

        while(current_time-start_time <= self.record_time):

            current_time = time.time()
            r.sleep()
            

        os.killpg(os.getpgid(self.process_audio.pid), signal.SIGINT)
        self.recording = False
        self.logger.log("Stop recording...")
        recorder_pub.publish(False)
        self.process_audio = None

def webcam_control(start_record):
    if start_record.data:
        # Start recording
        if not recorder.recording:
            recorder_pub.publish(True)
            recorder.start_record()

# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Log, then shut down remaining
    recorder.logger.log('STOP')
    recorder.record_time = 0
    #if process_audio is not None and process_audio.poll() is None:
    #    os.killpg(os.getpgid(process_audio.pid), signal.SIGTERM)
    rospy.signal_shutdown("User requested shutdown.")
    sys.exit(0)

if __name__ == '__main__':
    # Setup callbacks, ROS and logging
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    recorder = AudioRecorder()

    #recorder.logger.log('\n'.join(sys.path), printing = True)

    rospy.init_node('audio', anonymous=True)
    rospy.Subscriber("/usb/webcam", Bool, webcam_control, queue_size=1)
    recorder_pub = rospy.Publisher("/recording", Bool, queue_size=1)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()

    
    
