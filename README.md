# Workplace Companion Robot
This repository is the entire SHARE Lab's workplace companion project including all ros, python, and arduino code components.

This repository is a ROS package for SHARE Lab's Workplace Companion Robot project. To add this package to your existing ROS packages, enter your `ros_workspace/src` folder and `git clone https://github.com/shareresearchteam/workplace_companion_robot.git`. Return to your `ros_workspace` folder and run `catkin_make` for ROS to recognize the package within its paths. 

This package primarily uses Python files that will now be located in `ros_workspace/src/workplace_companion_robot/src/`. The primary files to look at will be `routines.py` and `controller.py`. `routines.py` contains the routine logic and actions, while `controller.py` contains a control loop. Generally speaking, `controller.py` will "tick" the current routine and change the routine if the conditions are suitable, and will properly start/stop routines as needed.

Please familiarize yourself with the structure of `routines.py`: the arrays of strings indicate lines of code that will be executed by the routine "ticking". Code will be executed in a serial fashion; that is, the next line of code will only execute when the previous has completed, which is determined by polling the robot's current actions for most. `wait(time in seconds)` is a custom defined command that does not block the control loop but effectively executes a `time.sleep(time in seconds)` statement for the robot.

Please let me know if you have any questions!

## TODO
- Quick tests for "expressive" checks
- Take another look at long vs short stand
- Take another look at snooze button function
- Look at hard animation stop for cozmo
- Look at making just a face or jsut face + head for goal or lifted states (remove lift or wheel motion)
- Look at Cozmo eyes closed during base/retention
- Update README
- Clean out deprecated code in focus tracker

## Required Extra Python Packages
- custom pycozmo
- pyserial
- pygame
- pyaudio
- wave
- opencv-python

## System Sensor States
Based on sensor set and no internal logic, there are 288 possible system states

### Seat Sensor
1. Seated
2. Standing
3. Transition Sit to Stand
4. Transition Stand to Sit

### Keyboard Sensor
1. Active, typing
2. Active, not typing
3. Not active, not typing

### Button
1. Pressed
2. Not Pressed

### Camera
1. Recording
2. Not Recording

### Cozmo
1. On charger, charging
2. On charger, idle
3. Off charger, idle
4. Off charger, getting attention
5. Off charger, success animation
6. Off charger, airborne

## Old

## Testing
- Save microphone audio during webcam recording
- Have Cozmo light color change on low battery
- Fix behavior where Cozmo wont act when charging
- Check recording behavior at right time
- Save Cozmo state on disconnect from robot/trying to reconnect

## Behavior
- Cozmo backpack color: 
    - Green when charging
    - Yellow when not charging
    - Red when NEED charging (< 3.5V battery)
- Demand breaks:
    - 2 hours is max before interrupt occurs
    - 1.5 hours is when Cozmo will look for break in keyboard activity ("inactive" state)
    - 5 minutes is snooze timer for interruption
    - 2 minutes without keyboard activity is "inactive"
- Start Recording:
    - 2 minutes before max timer
    - 2 minutes before snooze is up
    - When keyboard goes inactive during interrupt window
    - When user stands up


## Hardware
- Anki Cozmo robot
- USB Webcam
- i3 processor Win10 mini-pc with onboard wi-fi and bluetooth
- Chair sensor
- Keyboard sensor

## Setup

### Mini-PC Setup
1.  Flash Windows 10 version 1809 onto the mini-PC using Rufus
    - Do not connect to the internet during OS install
2. Run [Registry Mods](setup/registry_mods.ps1) as administrator to change to auto logon, maintain wi-fi, and stop automatic windows updates.
3. Connect keyboard sensor by USB and set to COM3 in the Device Manager
4. Connect bluetooth to chair sensor
5. Install Bluestacks
6. Install the Cozmo app on bluestacks using the app store
    - Confirm Cozmo connects to bluestacks
7. Setup Windows Subsystem for Linux (WSL) [LINK]()
8. Install VirtualBox
9. Install VirtualBox extension pack
10. Create an Ubuntu 18.04 VM in VirtualBox
    - Minimum 2 GB RAM
    - Minimum 30GB virtual hard disk space
    - Set bluetooth passthrough
    - Set serial passthrough on COM3
    - Set port forwarding:
        - Protocol: TCP
        - Host IP: 127.0.0.1
        - Host Port: 2522
        - Guest IP: 10.0.2.15
        - Guest Port: 22
11. Setup ssh for the VM and WSL
12. SSH into the virtual machine from WSL
13. Run:
    ```
cd ~
git clone https://github.com/shareresearchteam/workplace_companion_robot.git
cd workplace_companion_robot
cd setup
./vm_setup.sh
    ```
14. Install ADB on Windows, and WSL
15. Configure WSL bashrc
    ```
sudo echo "export PATH=${PATH}:~/android-sdk-linux/platform-tools" >> ~/.bashrc
sudo echo "adb connect localhost:5555" >> ~/.bashrc
sudo echo "# ssh to vm" >> ~/.bashrc
sudo echo "while true; do" >> ~/.bashrc
sudo echo "         ssh -p 2522 -R 5554:localhost:5554 -R 5555:localhost:5555 cozmo@127.0.0.1" >> ~/.bashrc
sudo echo "         sleep 1" >> ~/.bashrc
sudo echo "         continue" >> ~/.bashrc
sudo echo "done" >> ~/.bashrc
    ```
16. 
17. MORE SCREAM
18. Run [add startup tasks](setup/add_startup_tasks.ps1) as administrator to creat bluestacks schedules tasks and auto start the linux environment
19. Restart mini-PC

### Chair Sensor



