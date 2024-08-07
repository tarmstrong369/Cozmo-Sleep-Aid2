#!/usr/bin/env python3

import os
import datetime
import configparser
import pathlib

verbose = []
# Define logging verboseness
#with open(os.path.dirname(os.path.abspath(__file__))[0:-3] + 'options.cfg', 'r') as f:
#    for line in f.readlines():
#        verbose.append(line.strip())

config = configparser.ConfigParser()
config.read(str(pathlib.Path(__file__).resolve().parents[1] / "config.ini"))
if 'PrintLogging' in config:
    for key in config['PrintLogging']:
        if config['PrintLogging'].getboolean(key):
            verbose.append(str(key))

ID = config['SetupSettings']['SubjectID']

start_date = datetime.datetime.now()
if config.has_section("SaveState"):
    start_date = datetime.datetime.strptime(config['SaveState']['StartDate'],'%Y-%m-%d %H:%M:%S.%f')


phase_list = config['SetupSettings']['PhaseList']
phase_list = phase_list.split(", ")
phase_times_string = config['SetupSettings']['PhaseDurations'].split(", ")
phase_times = [datetime.timedelta(days=float(ele)) for ele in phase_times_string]


# Define subject information: id.subject should be changed for each subject
#with open(os.path.dirname(os.path.abspath(__file__))[0:-3] + 'id.subject', 'r') as f:
#    ID = f.readline().strip()
# Path format: identifier_YYYY-MM-DD.log
PATHFORMAT = '{0}_{1}_{2}_{3:04d}-{4:02d}-{5:02d}.log'
# Line format: HH:MM:SS-SSS, <string>
LINEFORMAT = '{0:02d}:{1:02d}:{2:02d}.{3:06d}, {4}\n'

class Logger:
    def __init__(self, source, start_date=start_date):
        self.source = source
        self.printing = self.source in verbose
        # Drops all logs in logs/logfiles/*.log
        self.folder_path = str((pathlib.Path(__file__).resolve().parents[1] / "logs" / "logfiles").resolve()) + "/"
        self.current_date = datetime.datetime.now()
        self.start_date = start_date
        self.elapsed_days = (self.current_date - self.start_date)
        self.study_phase = ""
        self.update_phase()

    def log(self, message, printing=False):
        # Determines the current datetime and defines the filename (for multi-day runs)
        dt = datetime.datetime.now()
        self.check_day()
        filename = self.folder_path + PATHFORMAT.format(ID, self.study_phase, self.source, dt.year, dt.month, dt.day)

        # Writes the message to the file according to the line format
        with open(filename, 'a') as file:
            line = LINEFORMAT.format(dt.hour, dt.minute, dt.second, dt.microsecond, message)
            file.write(line)

        if self.printing or printing:
            print(self.source + ": " + line, flush=True)#message)

    def update_phase(self):
        test_days = self.elapsed_days
        for i in range(len(phase_list)):
            if test_days > phase_times[i]:
                test_days = test_days - phase_times[i]
            else:
                self.study_phase = phase_list[i]
                return
        self.study_phase = 'end'

    def check_day(self):
        self.current_date = datetime.datetime.now()
        new_days = (self.current_date - self.start_date)
        check_delta = new_days - self.elapsed_days
        if check_delta > phase_times[0]:
            self.elapsed_days = new_days
            self.update_phase()

    
