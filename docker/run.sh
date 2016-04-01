#!/bin/sh

xhost +local:root
GPU=0

nvidia-docker \
  run \
	-it \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/${USER}/Downloads/data:/root/catkin_ws/src/ros_caffe/data" \
	--device /dev/bus/usb:/dev/bus/usb \
	--privileged \
	ros:indigo-ros-caffe-dev bash

xhost -local:root
