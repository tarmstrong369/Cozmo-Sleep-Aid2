# This (should be) all the setup and initialization components for setting up a the mini-pc from scratch, running Ubuntu 18.04.4 LTS.

echo "Starting setup from fresh empty Ubuntu 18.04.4 LTS image"
echo "This assumes we already have internet connection"
echo "This also assumes we can pull git repos"
echo ""
echo ""
echo "YOU WILL NEED TO ADJUST THE CONFIG.INI FILE AFTER SETUP COMPLETE"
echo "You will need to pick the correct bluetooth mac address for the chair sensor"
echo "You will also need to provide a participant id number"
echo ""
echo ""
echo "Starting setup! Please do not close or exit this setup!"
echo ""
echo ""

echo "###### DEPENDENCY INSTALLS ######"
echo "Includes:"
echo "   - ffmpeg"
echo "   - ROS melodic - modified to run python 3"
echo "   - opencv-python"
echo "   - pyserial"
echo "   - pygame"
echo "   - Custom pycozmo (until pull request is completed)"
echo ""
echo "Starting..."

echo "Installing ffmpeg for audio recording..."
# Install ffmpeg for audio record (video handled with open-cv)
apt-get --yes --force-yes install ffmpeg

echo "Starting ROS Melodic install..."
# Setup and install ROS
echo "Adding ROS to distro list..."
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
#apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
echo "Installing curl to get ROS keys..."
apt install curl
echo "Getting ROS keys..."
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
echo "Update system software..."
apt update
echo "Force install ROS Melodic, ROS python hooks, and python 3 upgrade..."
apt --yes --force-yes install ros-melodic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential python3-yaml python3 python3-pip python3-pil.imagetk

# Make changes and add other python requirements
echo "Update python 3 pip..."
pip3 install --upgrade --user pip
echo "Install python 3 catkin, ros, and calendars..."
pip3 install --user catkin_pkg rospkg pytz icalendar
echo "Install python 3 opencv, pyserial, and pygame..."
# Note, pyaudio and wave were removed from this install block, because I'm pretty sure we aren't actually using them at all
pip3 install --user opencv-python pyserial pygame
echo "Completed ROS installation!"
echo ""


echo "#### PyCozmo Install ####"
echo "Starting Pycozmo install from cusotm source code..."

# Get our current custom pycozmo (at least until changes are pulled to master)
echo "Putting pycozmo source code in home dir..."
cd ~
git clone git@github.com:reakain/pycozmo.git

# Install pycozmo from custom source
echo "Installing pycozmo from our source..."
cd ~/pycozmo
python3 setup.py install --user

# Get all the pycozmo assets (required for animations)
echo "Downloading pycozmo animations and resources..."
pycozmo_resources.py download

# Copy custom animations to pycozmo
echo "Copying our custom animations to the pycozmo animations dir..."
cp ~/workplace_companion_robot/custom_anims/*.bin ~/.pycozmo/assets/cozmo_resources/assets/animations/
echo "Completed installation of PyCozmo!"
echo ""


echo "####### SYSTEM INITIALIZATIONS #######"
echo "Starting final system preference setting and ROS initialization..."

# Setup ROS initialization and updates (finalizes our install and includes adding ROS commands to our environ)
cd ~
#pip3 install --user catkin_pkg rospkg 'cozmo[camera]' pytz icalendar
echo "Initializing and updating ROS..."
rosdep init && rosdep update
echo "Adding ROS setup to environ..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && source ~/.bashrc

# Make ROS Workspace and move system source
echo "Making ROS workspace ~/ros_ws ..."
mkdir -p ~/ros_ws/src
echo "Copying system source to ROS workspace..."
cp -R ~/workplace_companion_robot ~/ros_ws/src

# Build System Code in ROS
echo "Building system source with ROS..."
cd ~/ros_ws && catkin_make
source ~/ros_workspace/devel/setup.bash

# Setup serial permissions for the teensy
#echo 'KERNEL=="ttyS2", MODE="0666"' > /etc/udev/rules.d/keyboard-activity-serial-rule.rules
#chmod 666 /dev/ttyS2
echo "Setting serial port permissions for key activity sensor..."
cp ~/ros_ws/src/workplace_companion_robot/setup/00-teensy.rules /etc/udev/rules.d/

# Copy terminal auto start script to system config
echo "Adding terminal auto-launch on system start..."
cp ~/ros_ws/src/workplace_companion_robot/setup/terminal.desktop ~/.config/autostart/

# Add rc.local to /etc to handle blocking autosuspend of bluetooth
echo "Updating rc.local to prevent autosuspension of bluetooth..."
cp ~/ros_ws/src/workplace_companion_robot/setup/rc.local /etc/rc.local

# Add launch system to bash
echo "Adding system launch script to shell bash..."
echo "source ~/ros_ws/src/workplace_companion_robot/launch_system.bash" >> ~/.bashrc

# Setup id files
# THESE ARE NOW SET IN THE CONFIG.INI
echo "Making copy of base_config to use as working config..."
cp ~/ros_ws/src/workplace_companion_robot/base_config.ini ~/ros_ws/src/workplace_companion_robot/config.ini
echo "Config file located at ~/ros_ws/src/workplace_companion_robot/config.ini"
echo "Please remember to update subject ID and set the correct"
echo "chair sensor bluetooth address in the config file!"
echo ""
echo "All auto setup complete!"
echo ""
echo "Please press any button to exit"
read x
# Instead we'll just make a copy of base_config.ini
#echo "Webcam" > ~/ros_ws/src/workplace_companion_robot/id.webcam
#echo "1" > ~/ros_ws/src/workplace_companion_robot/id.subject
#hcitool con | grep -o "[[:xdigit:]:]\{11,17\}" > ~/ros_ws/src/workplace_companion_robot/id.chair_bluetooth

