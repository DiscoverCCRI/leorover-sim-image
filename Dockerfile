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
RUN cd ~/catkin_ws/src \ 
&& git clone https://github.com/LeoRover/leo_desktop.git \
&& git clone https://github.com/LeoRover/leo_simulator.git \
&& git clone https://github.com/LeoRover/leo_common.git

# Make the package for the API
RUN cd ~/catkin_ws/src \
&& git clone -b simulation https://github.com/DiscoverCCRI/RoverAPI.git 
RUN cd ~/catkin_ws/src \
&& catkin_create_pkg rover_api rospy roscpp geometry_msgs sensor_msgs 
RUN mkdir -p ~/catkin_ws/src/rover_api/src/rover_api/ \
&& cd ~/catkin_ws/src/RoverAPI \
&& mv camera_driver/discover_camera.py rover_driver/discover_rover.py ~/catkin_ws/src/rover_api/src/rover_api/ \
&& mv scripts/setup.py ~/catkin_ws/src/rover_api \
&& cd ~/catkin_ws/src/rover_api && mkdir scripts \
&& mv ~/catkin_ws/src/RoverAPI/scripts/example.py ~/catkin_ws/src/rover_api/scripts \
&& chmod u+x ~/catkin_ws/rover_api/scripts/example.py \
&& mkdir -p ~/scripts && mv ~/catkin_ws/src/RoverAPI/scripts ~/ \
&& cd ~/scripts && chmod u+x * && rm builder.sh \
&& cd ~/catkin_ws/src/rover_api && echo "catkin_python_setup()" >> CMakeLists.txt \
&& echo "catkin_install_python(PROGRAMS src/rover_api/discover_rover.py src/rover_api/discover_camera.py DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION})" >> CMakeLists.txt

# Clean
RUN sudo apt-get -y autoremove && sudo apt-get -y autoclean 
RUN rm -rf /var/lib/apt/lists/* && rm -r ~/catkin_ws/src/RoverAPI

# Build the catkin workspace
#RUN . /opt/ros/$ROS_DISTRO/setup.bash && cd ~/catkin_ws && catkin_make
