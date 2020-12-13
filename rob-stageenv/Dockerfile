FROM iocchi/stage_environments@sha256:2ff3c9991dfce983cb0d6f764b1708dca580fb44db03cec8fa15abbd044f80a7

WORKDIR /home/robot/src/stage_environments/
COPY rob.patch .
RUN git apply rob.patch

WORKDIR /home/robot/
RUN /bin/bash -ci "source $HOME/.bashrc; cd $HOME/ros/catkin_ws; catkin_make"
