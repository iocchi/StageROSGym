This docker just applies a small patch to the last pushed stage_environments.
Run:

    docker build -t iocchi/stage_environments:<my-version> .

And then add <my-version> as argument when launching stage_docker_create.bash
or nvidia-scripts/stage_docker_create.bash
