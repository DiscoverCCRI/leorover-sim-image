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
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6494C6D6997C215E
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 605C66F00D6C9793
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 0E98404D386FA1D9
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 648ACFD622F3D138
RUN sudo apt update && sudo apt -y install libu2f-udev
RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
RUN sudo dpkg -i google-chrome-stable_current_amd64.deb


# Update and get git
RUN apt-get update && apt-get -y full-upgrade && apt-get -y install git vim wget

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


# Get python dependencies
RUN wget https://bootstrap.pypa.io/get-pip.py
RUN python3 get-pip.py
RUN pip3 install opencv-python numpy matplotlib


# Get gazebo
RUN curl -sSL http://get.gazebosim.org | sh
RUN sudo apt-get -y install ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-ros-control ros-noetic-map-server
RUN sudo apt-get install -y libgazebo11-dev


# Setup catkin workspace
RUN cd ~/ && mkdir catkin_ws && cd ~/catkin_ws && mkdir src
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


# Install teleop keyboard tools
RUN sudo apt-get -y install ros-noetic-teleop-twist-keyboard \
ros-noetic-*-controller


# Get needed packages
RUN cd ~/catkin_ws/src \
&& git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git \ 
&& git clone https://github.com/LeoRover/leo_desktop.git \
&& git clone https://github.com/LeoRover/leo_simulator.git \
&& git clone https://github.com/LeoRover/leo_common.git \
&& git clone https://github.com/Slamtec/rplidar_ros.git \
&& git clone https://github.com/LeoRover/leo_navigation_tutorial.git \
&& git clone https://github.com/machinekoder/ar_track_alvar.git -b noetic-devel

# Setup rosdep
RUN cd ~/catkin_ws \
&& rosdep init \
&& rosdep update \
&& rosdep install --from-paths src -iy


# Make the package for the API
RUN git clone -b simulation https://github.com/DiscoverCCRI/RoverAPI.git \
&& mv RoverAPI/rover_api ~/catkin_ws/src 


# Get ARTags imported
RUN cd ~/ \
&& git clone https://github.com/mikaelarguedas/gazebo_models.git \
&& cd gazebo_models/ar_tags/scripts \
&& ./generate_markers_model.py -g /usr/share/gazebo-11/models \
&& cd ~/ \
&& rm -r gazebo_models


# Set up world and launch
RUN cd ~/RoverAPI \
&& mv gazebo/worlds/* /usr/share/gazebo-11/worlds \
&& mv gazebo/media/dem/* /usr/share/gazebo-11/media/dem \
&& mv gazebo/macros.xacro ~/catkin_ws/src/leo_common/leo_description/urdf \
&& mv gazebo/*.stl ~/catkin_ws/src/leo_common/leo_description/models 


# Clean
RUN sudo apt-get -y autoremove && sudo apt-get -y autoclean 
RUN rm -rf /var/lib/apt/lists/* && sudo rm -r ~/RoverAPI
RUN rm *.py *.deb
RUN rm -r ~/catkin_ws/src/rover_api/src/rover_api \
&& git clone http://github.com/DiscoverCCRI/RoverAPI.git \ 
&& mv RoverAPI/rover_api/src/rover_api ~/catkin_ws/src/rover_api/src \
&& sudo rm -r RoverAPI
RUN git clone https://github.com/DFKI-NI/rospy_message_converter.git \
&& mv rospy_message_converter ~/catkin_ws/src

# Build the catkin workspace
RUN . /opt/ros/$ROS_DISTRO/setup.bash && cd ~/catkin_ws && catkin_make
