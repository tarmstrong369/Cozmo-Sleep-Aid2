#!/usr/bin/env python3
from src import webcam_recorder
import os
import sys
import time
import signal

# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    print("User requested shutdown.")
    recorder.record_time = 0
    sys.exit(0)


if __name__ == '__main__':
    # Define global wrapper for sitting status
    #signal.signal(signal.SIGINT, keyboard_interrupt_callback)
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)

    recorder = webcam_recorder.WebcamRecorder()

    if not recorder.recording:
            recorder.start_record()
