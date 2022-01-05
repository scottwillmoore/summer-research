#!/bin/bash

docker run \
--name="baxter_mmc" \
--env="DISPLAY" \
--env QT_X11_NO_MITSHM=1 \
--volume="/run/user/1000/gdm:/run/user/1000/gdm" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
--hostname eric-docker-mmc \
--interactive \
--tty \
--gpus all \
--shm-size 2g \
--cap-add "SYS_ADMIN" \
baxter_mmc \
bash

#--volume="/home/eric/.gazebo/models:/home/scott/.gazebo/models" \
