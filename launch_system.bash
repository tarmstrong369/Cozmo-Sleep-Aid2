#!/bin/bash
if [[ ! $(pgrep rosout) ]]; then
    source ~/ros_ws/devel/setup.bash
    # Actual system launch file
    roslaunch workplace_companion_robot system_full.launch
    # For testing using a simulated seat sensor
    #roslaunch workplace_companion_robot serial_testing.launch
    # For testing using simulated seat sensor, key activity sensor, and snooze button
    #roslaunch workplace_companion_robot sim_testing.launch
fi
