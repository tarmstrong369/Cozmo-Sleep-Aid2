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
from std_msgs.msg import Bool

# TODO: Combine audio and video?
# TODO: pyaudio testing

class WebcamRecorder:
    recording = False
    
    def __init__(self, testing=False):
        config = configparser.ConfigParser()
        config.read(str(pathlib.Path(__file__).parents[1]) + '/config.ini')
        ID = config['SetupSettings']['SubjectID']
        audio_device = config['SetupSettings']['WebcamID']
        self.record_time = config['SetupSettings'].getfloat('RecordTime')
        # Convert to seconds
        self.record_time = self.record_time*60

        self.logger = logger_minimal.Logger('webcam')
        self.logger.log("START")

        # Path format: identifier_YYYY-MM-DD_HH:MM:SS.log
        self.PATHFORMAT = ID + '_{0:04d}-{1:02d}-{2:02d}-{3:02d}:{4:02d}:{5:02d}'

        self.folder_path = str(pathlib.Path(__file__).parents[1]) + '/logs/videos/'


    def start_record(self):
        video_thread = threading.Thread(target=self.record)
        video_thread.start()

    def record(self):
        self.recording = True
        self.logger.log("Start recording...")

        dt = datetime.datetime.now()
        filename = self.folder_path + self.PATHFORMAT.format(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)
        #filename = "test"

        # Video
        vid_capture = cv2.VideoCapture(0)
        if not vid_capture.isOpened():
            vid_capture.release()
            cv2.destroyAllWindows()
            self.logger.log("Failed to open webcam video stream")
            self.recording = False
            recorder_pub.publish(False)
            return

        frame_w = int(vid_capture.get(3))
        frame_h = int(vid_capture.get(4))

        #vid_cod = cv2.VideoWriter_fourcc(*'XVID')
        vid_cod = cv2.VideoWriter_fourcc('M','J','P','G')
        output = cv2.VideoWriter(filename + ".avi", vid_cod, 24.0, (frame_w, frame_h))

        # TODO: fix audio recording
        # Audio
        self.rate = 44100
        self.frames_per_buffer = 1024
        self.channels = 2
        self.format = pyaudio.paInt16
        self.audio_filename = filename + ".wav"
        #self.audio = pyaudio.PyAudio()
        #self.stream = self.audio.open(format=self.format,
        #                              channels=self.channels,
        #                              rate=self.rate,
        #                              input=True,
        #                              frames_per_buffer = self.frames_per_buffer)
        self.audio_frames = []

        #self.stream.start_stream()

        start_time = time.time()
        current_time = time.time()

        while(vid_capture.isOpened() and (current_time-start_time <= self.record_time)):
            # Capture each frame of webcam video
            ret,frame = vid_capture.read()
            if ret == True:
                output.write(frame)

            # Get the audio data
            #data = self.stream.read(self.frames_per_buffer) 
            #self.audio_frames.append(data)

            current_time = time.time()
            # Close and break the loop after pressing "x" key
            #if cv2.waitKey(1) &0XFF == ord('x'):
            #    break

        # close the already opened camera
        vid_capture.release()
        # close the already opened file
        output.release()
        # close the window and de-allocate any associated memory usage
        cv2.destroyAllWindows()
        self.logger.log("Finished recording...")

        #self.stream.stop_stream()
        #self.stream.close()
        #self.audio.terminate()
        #waveFile = wave.open(self.audio_filename, 'wb')
        #waveFile.setnchannels(self.channels)
        #waveFile.setsampwidth(self.audio.get_sample_size(self.format))
        #waveFile.setframerate(self.rate)
        #waveFile.writeframes(b''.join(self.audio_frames))
        #waveFile.close()

        self.recording = False
        recorder_pub.publish(False)

def webcam_control(start_record):
    global recorder
    if start_record.data:
        # Start recording
        if not recorder.recording:
            recorder = WebcamRecorder()
            recorder_pub.publish(True)
            recorder.start_record()

# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Log, then shut down remaining
    recorder.logger.log('STOP')
    recorder.record_time = 0
    rospy.signal_shutdown("User requested shutdown.")
    sys.exit(0)

if __name__ == '__main__':
    # Setup callbacks, ROS and logging
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    recorder = WebcamRecorder()

    #recorder.logger.log('\n'.join(sys.path), printing = True)

    rospy.init_node('webcam', anonymous=True)
    rospy.Subscriber("/usb/webcam", Bool, webcam_control, queue_size=1)
    recorder_pub = rospy.Publisher("/recording", Bool, queue_size=1)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()

    
    
