#!/bin/sh
# Install the prerequisites for the ROS exploring code

sudo apt-get install \
ros-noetic-openni-* \
ros-noetic-openni2-* 
ros-noetic-freenect-* 
ros-noetic-usb-cam \
ros-noetic-laser-*  \
ros-noetic-slam-gmapping \
ros-noetic-joystick-drivers 
ros-noetic-orocos-kdl  \
ros-noetic-python-orocos-kdl \
ros-noetic-dynamixel-motor-* \
ros-noetic-vision-opencv \
ros-noetic-depthimage-to-laserscan \
ros-noetic-arbotix-* \
ros-noetic-turtlebot-*
ros-noetic-move-base \
ros-noetic-map-server \
ros-noetic-fake-localization \
ros-noetic-amcl  \
ros-noetic-moveit-ros \
ros-noetic-ecto \
ros-noetic-manipulation-msgs \
ros-noetic-navigation \
ros-noetic-gazebo-ros-control \
ros-noetic-ros-controllers \
ros-noetic-ros-control \
ros-noetic-sound-play \
ros-noetic-slam-gmapping \
ros-noetic-moveit-visual-tools \
ros-noetic-moveit-simple-controller-manager \
ros-noetic-moveit-planners-ompl \
ros-noetic-moveit-fake-controller-manager \
ros-noetic-moveit-commander  \
python-rosinstall \
python-setuptools  \
python-opencv  \
libasound2-dev \
libopencv-dev git subversion mercurial

