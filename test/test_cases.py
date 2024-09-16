#!/usr/bin/env python3

# TODO: I don't know the state of this...

CASE_LIST = []

# Variables
# Chair sensor: sit, stand, transitioning to sit, transitioning to stand
# Keyboard sensor: active input, active no input, inactive input, inactive no input
# button: press, no press
# break timing: not time, within window, max time
# snooze state: snoozed, not snoozed

# Issues: no immediate access to trick system on time? This mucks up our ability to test... 
# well, really anything in a consistent manner. We also can't guarantee when the system will 
# check the time, so we can't change the clock a bunch to account for it.

# Might be easier to just make a fake user.... 

is_typing = True
is_button_press = True
is_sitting = True
