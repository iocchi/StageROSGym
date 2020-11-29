# StageROSGym

Repository to run experiments with ROS Stage simulator and Open AI Gym environment.

The `StageROSGym` environment is based on Stage ROS simulator `stage_environments` 
and on robot behaviors implemented in `rc-home-edu-learn-ros`.
These functionalities are available through docker containers that are pulled by the script `stagerosgym.bash` provided in this repository.

## Quick start

To start all the processes, use the following command

    ./stagerosgym.bash start

Note: in the first run, this command will pull some docker images from Docker Hub.



