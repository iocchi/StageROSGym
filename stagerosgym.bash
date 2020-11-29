#!/bin/bash

# Manage stage_environments simulation
# Use: ./stagerosgym.bash  [start|stop|status|pause|resume|speedup <value>|footprints]

if [ "x$1" == "xstatus" ]; then

echo "====================================="
echo "Status"
echo ""

echo "Docker"
docker ps
echo "---------"
echo ""

echo "ROS"
docker exec -it rchomeedu-1804-melodic bash -ci "rosnode list"
echo "---------"
echo ""

echo "====================================="
echo ""

elif [ "x$1" == "xstart" ]; then

# stage_environments

echo "====================================="
echo "Starting stage_environments ..."

if [ ! -f stage_docker_create.bash ]; then

wget -N https://bitbucket.org/iocchi/stage_environments/raw/master/docker_create.bash
mv docker_create.bash stage_docker_create.bash
source stage_docker_create.bash

fi

docker start stage_environments

echo "stage_environments running"
echo "====================================="
echo ""


# rchome_edu


echo "====================================="
echo "Starting rc-home-edu-learn-ros ..."

if [ ! -f rchomeedu_docker_create.bash ]; then

wget -N wget https://raw.githubusercontent.com/robocupathomeedu/rc-home-edu-learn-ros/master/docker/1804/create.bash
mv create.bash rchomeedu_docker_create.bash
source rchomeedu_docker_create.bash

fi

docker start rchomeedu-1804-melodic

echo "rc-home-edu-learn-ros running"
echo "====================================="
echo ""


sleep 5

echo "====================================="
echo "Starting simulation"

s1=stage_environments

#docker exec -it stage_environments tmux new-window -t "$s1" -n "1"

docker exec -it stage_environments tmux send-keys -t "$s1:0" "rosrun stage_environments start_simulation.py DISB1" C-M

echo "Simulation running"
echo "====================================="
echo ""

sleep 5

echo "====================================="
echo "Starting robot behaviors"

s2=rchomeedu


docker exec -it rchomeedu-1804-melodic tmux new-session -d -s "$s2"
docker exec -it rchomeedu-1804-melodic tmux send-keys -t "$s2:0"   "cd ~/src/marrtino_apps/navigation" C-M
docker exec -it rchomeedu-1804-melodic tmux send-keys -t "$s2:0"  "roslaunch obstacle_avoidance.launch" C-M

echo "Robot behaviors running"
echo "====================================="
echo ""


elif [ "x$1" == "xstop" ]; then

echo "====================================="
echo "Stopping stagerosgym application...."

docker exec -it stage_environments tmux send-keys -t "$s1:0" "rosrun stage_environments quit.sh" C-M

docker exec -it rchomeedu-1804-melodic tmux send-keys -t "$s2:0" C-c

sleep 3

docker stop rchomeedu-1804-melodic
docker stop stage_environments

echo "...done"
echo "====================================="
echo ""



elif [ "x$1" == "xpause" ]; then

echo "====================================="
echo "Simulation paused"
docker exec -it stage_environments bash -ci "rostopic pub stageGUIRequest std_msgs/String   \"data: 'pause'\" --once"
echo "====================================="
echo ""



elif [ "x$1" == "xresume" ]; then

echo "====================================="
echo "Simulation resumed"
docker exec -it stage_environments bash -ci "rostopic pub stageGUIRequest std_msgs/String   \"data: 'resume'\" --once"
echo "====================================="
echo ""



elif [ "x$1" == "xspeedup" ]; then

SPEEDUPVALUE=1.0
if [ ! "$2" == "" ]; then
  SPEEDUPVALUE=$2
fi

docker exec -it stage_environments bash -ci "rostopic pub stageGUIRequest std_msgs/String   \"data: 'speedup_$SPEEDUPVALUE'\" --once"


elif [ "x$1" == "xfootprints" ]; then

docker exec -it stage_environments bash -ci "rostopic pub stageGUIRequest std_msgs/String   \"data: 'footprints'\" --once"

else

echo "Use: $0 [start|stop|status|pause|resume|speedup <value>|footprints]"

fi


