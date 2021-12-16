#!/bin/bash

docker run \
--name="baxter" \
--env="DISPLAY" \
--env QT_X11_NO_MITSHM=1 \
--volume="/run/user/1000/gdm:/run/user/1000/gdm" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
--hostname eric-docker \
--interactive \
--network baxter-net \
--ip 192.168.1.128 \
--tty \
--gpus all \
baxter \
bash
