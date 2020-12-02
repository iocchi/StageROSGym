#!/bin/bash

# Source this in current bash. It will declare two variables:
#  HAS_NVIDIA if the nvidia drivers have been found
#  NVIDIA_STR for the Docker arguments required for nvidia drivers

# Simple check
[ -d /usr/lib/nvidia* ]
HAS_NVIDIA=$?

# nvidia-docker2 greately simplifies GPU support
dpkg-query -s nvidia-docker2 &> /dev/null
if [ $? -eq 1 ]; then
	echo "Err: nvidia-docker2 not installed"
	exit 1
fi

if [ $HAS_NVIDIA ]; then
  NVIDIA_STR=' --gpus all,"capabilities=display" '
fi
