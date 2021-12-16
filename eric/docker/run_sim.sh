#!/bin/bash

docker run \
--name="test" \
--env="DISPLAY" \
--env QT_X11_NO_MITSHM=1 \
--env ROS_MASTER_URI=http://localhost:11311 \
--env ROS_IP=172.17.0.2 \
--volume="/run/user/1000/gdm:/run/user/1000/gdm" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
--volume="/home/eric/.gazebo/models:/home/scott/.gazebo/models" \
--hostname eric-docker-sim \
--interactive \
--tty \
--gpus all \
baxter_sim_nvidia \
bash


#--shm-size 2g \
#--cap-add "SYS_ADMIN" \