#!/bin/bash

# Manage stage_environments simulation
# Use: ./stagerosgym.bash  [start|start-dev|stop|status|pause|resume|speedup <value>|footprints]
#
# Note: start-dev was added to run containers on PC with nvidia options:
# I don't want to break the config for robot

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

elif [[ $1 == start || $1 == start-dev ]]; then

	# stage_environments

	echo "====================================="
	echo "Starting stage_environments ..."


	if [ `docker image ls | grep iocchi/stage_environments | wc -l` == 0 ]; then

		docker pull iocchi/stage_environments

	fi

	# No modifications
	if [[ $1 == start ]]; then
		if [ `docker container ls | grep iocchi/stage_environments | wc -l` == 0 ]; then

		wget -N https://bitbucket.org/iocchi/stage_environments/raw/master/docker_create.bash
		mv docker_create.bash stage_docker_create.bash
		chmod a+x stage_docker_create.bash
		./stage_docker_create.bash

		fi

	# Modified script for development
	elif [[ $1 == start-dev ]]; then

		bash nvidia-scripts/stage_docker_create.bash
		
	fi

	docker start stage_environments

	sleep 3

	if [ `docker ps | grep stage_environments | wc -l` == 0 ]; then

		echo "ERROR Container stage_environments not running"
		echo "====================================="
		echo ""

		exit 1

	fi

	echo "stage_environments running"
	echo "====================================="
	echo ""


	# rchome_edu


	echo "====================================="
	echo "Starting rc-home-edu-learn-ros ..."

	if [ `docker image ls | grep iocchi/rchomeedu-1804-melodic | wc -l` == 0 ]; then

		docker pull iocchi/rchomeedu-1804-melodic

	fi

	# No modifications
	if [[ $1 == start ]]; then
		if [ `docker container ls | grep iocchi/rchomeedu-1804-melodic | wc -l` == 0 ]; then
		
			wget -N https://raw.githubusercontent.com/robocupathomeedu/rc-home-edu-learn-ros/master/docker/1804/create.bash
			mv create.bash rchomeedu_docker_create.bash
			chmod a+x rchomeedu_docker_create.bash
			./rchomeedu_docker_create.bash
		
		fi

	# Modified script for development
	elif [[ $1 == start-dev ]]; then

		bash nvidia-scripts/rchomeedu_docker_create.bash
		
	fi

	docker start rchomeedu-1804-melodic

	sleep 3

	if [ `docker ps | grep rchomeedu-1804-melodic | wc -l` == 0 ]; then

		echo "ERROR Container rchomeedu-1804-melodic not running"
		echo "====================================="
		echo ""
		exit 1

	fi

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

	echo "Use: $0 [start|start-dev|stop|status|pause|resume|speedup <value>|footprints]"

fi


