# StageGym - RL with Stage ROS
# ROS Melodic, navigation, perception & additional packages
# Python3 OpenAIGym
# marrtino_apps

FROM ros:melodic-ros-base-bionic

ARG MACHTYPE=default

ARG DEBIAN_FRONTEND=noninteractive

###### User root ######

# install libraries and ros packages 

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
        tmux less sudo eom nano htop \
        wget iputils-ping net-tools openssh-client nginx \
        cmake make g++ git \
        libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
        mesa-common-dev mesa-utils freeglut3-dev \
        python3 python3-pip python3-setuptools \
        libwebsockets-dev \
        ros-melodic-desktop ros-melodic-move-base && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# User: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo

RUN adduser robot audio
RUN adduser robot video
RUN adduser robot dialout


###### User robot ######

USER robot

# Configuration

RUN echo "set -g mouse on" > $HOME/.tmux.conf 


# Python packages

RUN pip3 install \
    pandas scipy matplotlib sklearn jupyter \
    gym pygame

#    tensorflow keras keras-rl \

# Init ROS workspace

#RUN mkdir -p $HOME/ros/catkin_ws/src
#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd $HOME/ros/catkin_ws/src; catkin_init_workspace; cd ..; catkin_make"
#RUN echo "source \$HOME/ros/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
#RUN rosdep update
#RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"

RUN echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc

RUN mkdir -p $HOME/src && \
    cd $HOME/src && \
    git clone https://bitbucket.org/iocchi/marrtino_apps.git

RUN echo "export MARRTINO_APPS_HOME=\$HOME/src/marrtino_apps" >> $HOME/.bashrc
RUN echo "export ROBOT_TYPE=stage" >> $HOME/.bashrc


# Set working dir and container command

WORKDIR /home/robot

CMD ["/usr/bin/tmux", "new", "-s", "stagerosgym"]

