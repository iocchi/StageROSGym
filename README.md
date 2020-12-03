# StageROSGym

Repository to run experiments with ROS Stage simulator and Open AI Gym environment.

The `StageROSGym` environment is based on Stage ROS simulator `stage_environments` (https://bitbucket.org/iocchi/stage_environments/)
and on robot behaviors implemented in `rc-home-edu-learn-ros` (https://github.com/robocupathomeedu/rc-home-edu-learn-ros).
These functionalities are available through docker containers that are pulled by the script `stagerosgym.bash` provided in this repository.

## Quick start

1. Build the image for StageROSGym

    ./build.bash

2. Start all the processes and the simulator

    ./stagerosgym.bash start


3. Run StageROSGym container

    ./run.bash

4. Start a script

    In the `stagerosgym` container

        cd StageROSGym
        python testenv.py
 

5. Quit everything

    In the `stagerosgym` container, exit from all the windows or use `CTRL-b d`

    In the host system:

        ./stagerosgym.bash stop



## Additional information

Note: in the first run, this command will pull some docker images from Docker Hub.

Note: if you need special configuration (e.g., Nvidia drivers), you need to build docker images
following instructions in the above repositories.

Use

    ./stagerosgym.bash [start [--no_gui]|start-dev [--no_gui]|stop|status|pause|resume|speedup <value>|footprints]

        start                 start the docker containers and the simulator with GUI
        start --no_gui        start the docker containers and the simulator without GUI
				start-dev [--no_gui]  start with development options: nvidia drivers without devices
        stop						      stop the docker containers
        status                show running modules
        pause                 simulation pause
        resume                simulation resume
        speedup <v>           simulation speedup <v>
        footprints            show footprints


