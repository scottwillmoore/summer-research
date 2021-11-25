# Install Ubuntu

ARG UBUNTU_DISTRIBUTION="trusty"

FROM ubuntu:${UBUNTU_DISTRIBUTION}

# Install keys and sources for ROS

ARG ROS_DISTRIBUTION="indigo"
ARG ROS_SNAPSHOT="final"

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
    echo "deb http://snapshots.ros.org/${ROS_DISTRIBUTION}/${ROS_SNAPSHOT}/ubuntu trusty main" > /etc/apt/sources.list.d/ros.list

# Install dependencies for ROS

RUN apt-get update && \
    apt-get install -y \
        python-rosdep \
        python-rosinstall \
        python-vcstools
        
# Install ROS

ARG ROS_PACKAGE="desktop-full"

RUN apt-get update && \
    apt-get install -y \
        ros-${ROS_DISTRIBUTION}-${ROS_PACKAGE}

# Setup dependencies for ROS

RUN rosdep init && \
    rosdep update --include-eol-distros

# Create a ROS workspace for Baxter

RUN . /opt/ros/indigo/setup.sh && \
    mkdir -p /opt/baxter/src && \
    cd /opt/baxter && \
    catkin_make && \
    catkin_make install

# Install dependencies for Baxter

RUN apt-get update && \
    apt-get install -y \
        python-argparse \
        python-wstool
    
# Install Baxter
    
RUN . /opt/ros/indigo/setup.sh && \
    cd /opt/baxter/src && \
    wstool init . && \
    wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall && \
    wstool update

# Set the default locale

ARG LOCALE="en_US.UTF-8"

RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Install dependencies for the user

RUN apt-get update && \
    apt-get install -y \
        curl \
        less \
        sudo \
        unzip \
        vim \
        wget \
        zip

# Create the user

ARG USERNAME="scott"

RUN useradd -m ${USERNAME} 

USER ${USERNAME}

# Create a ROS workspace for the user

RUN . /opt/baxter/devel/setup.sh && \
    mkdir -p ~/workspace/src && \
    cd ~/workspace && \
    catkin_make && \
    catkin_make install

# Source the workspace for the user

RUN echo "source ~/workspace/devel/setup.bash" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=\"\"" >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=\"\"" >> ~/.bashrc
