#!/bin/sh
# Install the prerequisites for the ROS exploring code

sudo apt-get install \
ros-kinetic-openni-* \
ros-kinetic-openni2-* 
ros-kinetic-freenect-* 
ros-kinetic-usb-cam \
ros-kinetic-laser-*  \
ros-kinetic-slam-gmapping \
ros-kinetic-joystick-drivers 
ros-kinetic-orocos-kdl  \
ros-kinetic-python-orocos-kdl \
ros-kinetic-dynamixel-motor-* \
ros-kinetic-vision-opencv \
ros-kinetic-depthimage-to-laserscan \
ros-kinetic-arbotix-* \
ros-kinetic-turtlebot-*
ros-kinetic-move-base \
ros-kinetic-map-server \
ros-kinetic-fake-localization \
ros-kinetic-amcl  \
ros-kinetic-moveit-ros \
ros-kinetic-ecto \
ros-kinetic-manipulation-msgs \
ros-kinetic-navigation \
ros-kinetic-gazebo-ros-control \
ros-kinetic-ros-controllers \
ros-kinetic-ros-control \
ros-kinetic-sound-play \
ros-kinetic-slam-gmapping \
ros-kinetic-moveit-visual-tools \
ros-kinetic-moveit-simple-controller-manager \
ros-kinetic-moveit-planners-ompl \
ros-kinetic-moveit-fake-controller-manager \
ros-kinetic-moveit-commander  \
python-rosinstall \
python-setuptools  \
python-opencv  \
libasound2-dev \
libopencv-dev git subversion mercurial

