#!/usr/bin/env python3
from enum import Enum

class UserBreakState(Enum):
    NoBreak = 1
    NeedCozmow = 2
    NeedCozmob = 3
    # BreakRequest = 3
    # OnBreak = 4
    # BreakEnded = 5
    SnoozeTriggeredw1 = 4
    SnoozeTriggeredw2 = 5
    SnoozeTriggeredw3 = 6
    SnoozeTriggeredb1 = 7
    SnoozeTriggeredb2 = 8
    SnoozeTriggeredb3 = 9

