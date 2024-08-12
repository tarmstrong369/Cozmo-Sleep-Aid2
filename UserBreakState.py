#!/usr/bin/env python3
from enum import Enum

class UserBreakState(Enum):
    NoBreak = 1
    NeedCozmo = 2
    BreakRequest = 3
    OnBreak = 4
    BreakEnded = 5
    SnoozeTriggered = 6
