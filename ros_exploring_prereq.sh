#!/bin/sh
# Install the prerequisites for the ROS exploring code

sudo apt-get -y install \
ros-melodic-openni-* \
ros-melodic-openni2-* \
ros-melodic-usb-cam \
ros-melodic-laser-*  \
ros-melodic-slam-gmapping \
ros-melodic-joystick-drivers \
ros-melodic-orocos-kdl  \
ros-melodic-python-orocos-kdl \
ros-melodic-vision-opencv \
ros-melodic-depthimage-to-laserscan \
ros-melodic-arbotix-* \
ros-melodic-turtlebot-* \
ros-melodic-move-base \
ros-melodic-map-server \
ros-melodic-fake-localization \
ros-melodic-amcl  \
ros-melodic-moveit-ros \
ros-melodic-navigation \
ros-melodic-gazebo-ros-control \
ros-melodic-ros-controllers \
ros-melodic-ros-control \
ros-melodic-sound-play \
ros-melodic-slam-gmapping \
ros-melodic-moveit-visual-tools \
ros-melodic-moveit-simple-controller-manager \
ros-melodic-moveit-planners-ompl \
ros-melodic-moveit-fake-controller-manager \
ros-melodic-moveit-commander  \
python-rosinstall \
python-setuptools  \
python-opencv  \
libasound2-dev \
libopencv-dev git subversion mercurial

sudo apt-get -y install libboost-python-dev libboost-filesystem-dev libboost-system-dev \
        libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen \
        python-sphinx

