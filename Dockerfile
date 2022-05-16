FROM dorowu/ubuntu-desktop-lxde-vnc:focal-lxqt
MAINTAINER Cody Beck <cjb873@nau.edu>

# set environment variables
ENV ROS_DISTRO=noetic

#set shell to bash
SHELL ["/bin/bash", "-c"]

# add sources
RUN echo "deb http://archive.ubuntu.com/ubuntu bionic main universe" >> /etc/apt/sources.list
RUN echo "deb http://archive.ubuntu.com/ubuntu bionic-security main universe" >> /etc/apt/sources.list
RUN echo "deb http://archive.ubuntu.com/ubuntu bionic-updates main universe" >> /etc/apt/sources.list

# Update and get git
RUN apt-get update && apt-get -y upgrade && apt-get -y install git vim

# ROS Install
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt-get update && sudo apt-get -y install ros-noetic-desktop

# Setup ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Get dependencies
RUN sudo apt-get -y install \
python3-rosdep python3-rosinstall python3-rosinstall-generator \
python3-wstool build-essential

# Bootstrap rosdep
RUN sudo rosdep init && rosdep update

# Clean
RUN sudo apt-get -y autoremove && sudo apt-get -y autoclean 
RUN rm -rf /var/lib/apt/lists/*

# Get gazebo
RUN curl -sSL http://get.gazebosim.org | sh
RUN sudo apt-get -y install ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-ros-control
RUN sudo apt-get install -y libgazebo11-dev

# Setup catkin workspace
RUN cd ~/ && mkdir catkin_ws && cd ~/catkin_ws && mkdir src
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install teleop keyboard tools
RUN sudo apt-get -y install ros-noetic-teleop-twist-keyboard \
ros-noetic-*-controller

# Get needed packages
RUN cd ~/catkin_ws/src \
&& git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git 
#RUN . /opt/ros/$ROS_DISTRO/setup.bash && cd ~/catkin_ws && catkin_make
RUN cd ~/catkin_ws/src \ 
&& git clone https://github.com/LeoRover/leo_desktop.git \
&& git clone https://github.com/LeoRover/leo_simulator.git \
&& git clone https://github.com/LeoRover/leo_common.git

# Make the package for the API
RUN cd ~/catkin_ws/src \
&& git clone https://github.com/DiscoverCCRI/RoverAPI.git \
&& catkin_create_pkg rover_api rospy roscpp geometry_msgs sensor_msgs \
&& mv RoverAPI/camera_driver RoverAPI/rover_driver rover_api/src \
&& rm -r RoverAPI

# Build the catkin workspace
#RUN . /opt/ros/$ROS_DISTRO/setup.bash && cd ~/catkin_ws && catkin_make
