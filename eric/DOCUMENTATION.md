## wifi issues
>https://libredd.it/r/LenovoLegion/comments/p5vp05/my_journey_to_get_linux_on_legion_5_15ach6h/

get driver from github

There is a script to run when kernel changes, need ethernet to pull from git

# networking
```
nmcli connection show
nmcli connection modify
```
# docker

## docker build

-f for different dockerfile name, defaults to Dockerfile
-t for image name

## docker run
```
sudo docker run --env="DISPLAY" --env="XAUTHORITY" --volume="/run/user/1000/gdm:/run/user/1000/gdm" --volume="/tmp/.X11-unix:/tmp/.X11-unix" --interactive --net=host --rm --tty baxter bash



sudo docker run --env="DISPLAY" --env QT_X11_NO_MITSHM=1 --volume="/run/user/1000/gdm:/run/user/1000/gdm" --volume="/tmp/.X11-unix:/tmp/.X11-unix" --interactive --rm --tty --network host baxter bash
```

### open another terminal
```
sudo docker exec -it `sudo docker ps -q -l` bash
```

### open exited
```
sudo docker start -a -i `sudo docker ps -q -l`
```

## docker network
### may have slow download speeds? so does the wifi sharing method
```
docker network create -d macvlan \
  --subnet=192.168.1.0/24 \
  --gateway=192.168.1.1 \
  --ip-range=192.168.1.128/25
  -o parent=eno1 \
  baxter-net
```

## env variables
```
export ROS_MASTER_URI=http://baxter.local:11311

export ROS_IP=192.168.1.x

export ROS_HOSTNAME=eric-laptop-ubuntu
```

## Dockerfile changes

### adding additional download mirrors
```
COPY sources.list /etc/apt/
```

### Create the user
```
ARG USERNAME="scott"

RUN useradd -m ${USERNAME} && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # Replace 1000 with your user/group id
    usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME

USER ${USERNAME}
```





### packages not found, need to catkin_make but cannot find command:

### root shell
`sudo -s /bin/bash`

### source if needed, then run catkin_make as root







# getting Gazebo to work
```
export ROS_HOSTNAME=eric-laptop-ubuntu

cp opt/baxter/src/baxter/baxter.sh ~/workspace
```

libGL error: failed to load driver: swrast
install nvidia driver:
```
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:graphics-drivers 
sudo apt install nvidia-352 -y
```
### gui issue
### nvidia-docker opengl image
https://gitlab.com/nvidia/container-images/samples/-/blob/master/opengl/ubuntu14.04/glxgears/Dockerfile
https://hub.docker.com/r/nvidia/opengl/tags?page=1&name=ubuntu14.04
https://gitlab.com/nvidia/container-images/opengl
https://github.com/NVIDIA/nvidia-docker/issues/11#issuecomment-236379456


### bind mount for downloaded gazebo models
`--volume="/home/eric/.gazebo/models:/home/scott/.gazebo/models" \`

### give user ownership of .gazebo folder
from user directory:  
`sudo chown scott:scott .gazebo/`

## control robot with moveit python interface

### cannot fetch robot state
https://github.com/ros-planning/moveit/issues/1187#issuecomment-509216732
### baxter publishes to non-standard rostopic /robot/joint_states, while moveit subscribes to joint_states
### remap topic

### in python:
```
joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
```

### .launch file xml:
`<remap from="joint_states" to="robot/joint_states"/>`  
baxter_gripper.launch  
planning_context.launch  
move_group.launch  
	- planning_pipeline.launch  
moveit_rviz.launch  
default_warehouse_db.launch  
	- warehouse.launch  
		-- warehouse_settings.launch  
		
### cannot fetch robot state (physical robot)
### need to synchronise time between robot and workstation
### check offset:
`ntpdate -q <robot-ip>`
### ^ causes problems with local machine NTP, get systemd NTP service instead
### change time on baxter - go to robot field service menu (Alt FF during boot)

### core dump terminate error
https://github.com/ros-planning/moveit_commander/issues/15

### baxter's ikfast plugins do not work as is
[ERROR] [1638922548.688147125]: Unknown marker name: 'EE:goal_left_gripper' (not published by RobotInteraction class)

## abandon moveit

# MMC

## need python 3.6+ for robotics toolbox library
### chose 3.9.9, most recent accepted version
### need to install from source in the ubuntu 14.04 container
http://devopspy.com/python/install-python-3-6-ubuntu-lts/
### but need openssl 1.0.2+, had 1.0.1f by apt install methods
### install openssl from source
https://stackoverflow.com/questions/53543477/building-python-3-7-1-ssl-module-failed
### ****worked? have to combine the two recommendations
### sketchy installation path
`/opt/python-3.9.9`  
go to `/bin/` for pip3 and python3/3.9 executables  
the roboticstoolbox package is located at `/home/scott/.local/lib/python3.9/site-packages/roboticstoolbox/`
### change PATH env variable (add to bashrc)
original: `/opt/ros/indigo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin`  
`export PATH=/opt/python-3.9.9/bin:$PATH`

### old stuff that didn't work
>~~### build on local machine (ubuntu 20.04) and move into container~~
>~~https://stackoverflow.com/questions/64333458/python3-8-on-ubuntu-14-04~~
>~~`docker cp <SRC_PATH> <CONTAINER>:<DEST_PATH>`~~
>~~### unzip, then~~
>~~sudo make install~~
>~~### don't think that actually works, use a ppa instead~~
>~~### deadsnakes ppa~~
>~~https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa~~
>~~### 3.7 is latest for trusty (14.04)~~
>~~### don't do install python3.7, this version has no pip or ensurepip to get it~~
>~~https://github.com/deadsnakes/issues/issues/79~~
>~~`sudo apt install python3.7-venv`~~
>~~### then get pip with ensurepip~~
>~~https://stackoverflow.com/questions/44761958/using-pip3-module-importlib-bootstrap-has-no-attribute-sourcefileloader~~
>~~`python3.7 -m ensurepip --upgrade`~~
>~~### this makes pip available~~
>~~`python3.7 -m pip ...`~~
>~~### this removes python3-pip so might mess with the existing python3.4~~
>~~### but hopefully it does not cause trouble~~  

## installing roboticstoolbox
> error: ‘for’ loop initial declarations are only allowed in C99 mode

> https://github.com/numpy/numpy/issues/14182


## swift location
`vim /home/scott/.local/lib/python3.9/site-packages/swift/Swift.py`

https://stackoverflow.com/questions/35313876/after-installing-with-pip-jupyter-command-not-found

## running firefox in container
needs higher shared memory, defaults to 64m, 2g seems to work
### for running containers:
>https://github.com/docker/cli/issues/1278
### for new containers:
`docker run --shm-size 2g ...`
### need new version (used 95) for swift

## ipv6 resolution with swift
line 109 in /home/scott/.local/lib/python3.9/site-packages/swift/SwiftRoute.py:  
```
start_server = websockets.serve(self.serve, "localhost", port)
```  
"localhost" is converted to "::1" in ipv6, can't be resolved in container

## need to enable ipv6 for docker
>https://docs.docker.com/config/daemon/ipv6/  

**RESTART instead of reload at the end**

# adding baxter to roboticstoolbox
## urdf and meshes are in ros package baxter_description

## inside the baxter's urdf xacro files: 
>No such file or directory: /home/scott/.local/lib/python3.9/site-packages/rtbdata/xacro/baxter_description/robot_urdf/../robot_urdf/../urdf/electric_gripper/rethink_electric_gripper.xacro [Errno 2] No such file or directory: '/home/scott/.local/lib/python3.9/site-packages/rtbdata/xacro/baxter_description/robot_urdf/../robot_urdf/../urdf/electric_gripper/rethink_electric_gripper.xacro'  

The xacro files use $(find) which is from ROS and does not work once the package has been moved. Packages also cannot be built again since roboticstoolbox has nothing to do with ROS. Need to specify path explicitly instead. This is system specific.  

Paths that need to be changed are found throughout the urdf descriptions in the python pacakges install path, under rtbdata. For this system it is:  
```
/home/scott/.local/lib/python3.9/site-packages/rtbdata/xacro/
```
Also need to specify "baxter_description" in the URDF_read method. all urdf and xacro files need to be under this directory. 

## solution:
1. put all files under rethink_ee_description under baxter_description, merging the mesh and robot folders  
2. change paths accordingly, replace all instances of rethink_ee_description in xacro files with baxter_description
3. the total number of links should be 57, adjust the Baxter.py file accordingly with base_link and gripper_links (link lists can be visualised more easily in python debug mode)  
values are: 0 for base, 50 for left_gripper, 56 for right_gripper


## ALTERNATIVELY: just use the non-xacro .urdf file
need to run rosrun xacro from baxter.urdf

https://github.com/elgalu/docker-selenium/issues/58
https://github.com/SeleniumHQ/docker-selenium/issues/487

https://stackoverflow.com/questions/38758627/how-can-we-add-capabilities-to-a-running-docker-container

`docker run --cap-add "SYS_ADMIN"`


## baxter neutral position:
position: [-4.875246055746629e-07, -1.5394258110465822e-10, -0.020833000269383728, -1.189972011753838, 1.9400294787022645, -0.0800000014515021, -0.9999845805893166, 0.6699967180466144, 1.030008744611373, -0.4999997026333549, 0.020833298459408452, 1.2980118078276757e-10, 1.1899720108865521, 1.9400294782207883, 0.08000000114257322, -0.9999845809809731, -0.6699967427602509, 1.0300087584583828, 0.49999971413654354]

velocity: [7.941592711257444e-09, 6.652573145366595e-08, 7.429526652220791e-08, 1.9804943839737066e-07, -5.510188023963251e-07, -1.478122832530591e-07, 7.097667963182467e-08, -4.1834909383977015e-07, 1.3434520386184567e-06, 1.62558167063411e-06, 0.00029540079316973754, -3.102581863230147e-06, -9.480175649467892e-07, -1.2135871376982062e-06, 7.221916352377831e-08, -2.4014901668267125e-07, -2.4424297093226555e-05, 1.5578684010889564e-05, 9.741585299037481e-06]

effort: [0.0, 0.0, 0.0, -0.12594652471431544, -0.16213462408831703, 5.907120836923241e-06, -0.15419778272018902, 0.0032815425197796344, -0.007868954907239711, -0.0002957618890153668, 0.0, 0.0, 0.12594695195211614, -0.16213571247458702, -4.17672803010305e-06, -0.15419613346168148, -0.0032816733203411275, -0.007868604352685438, 0.0002956330325076806]

### joints in roboticstoolbox and in ros are specified in different order:
```
rostopic joint names: ['head_pan', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
```
for what these names represent, refer to https://sdk.rethinkrobotics.com/wiki/Arms for diagram 
```        
roboticstoolbox joint names: ['head', 'right_upper_shoulder', 'right_lower_shoulder', 'right_upper_elbow', 'right_lower_elbow', 'right_upper_forearm', 'right_lower_forearm', 'right_wrist', 'r_gripper_l_finger', 'r_gripper_r_finger', 'left_upper_shoulder', 'left_lower_shoulder', 'left_upper_elbow', 'left_lower_elbow', 'left_upper_forearm', 'left_lower_forearm', 'left_wrist', 'l_gripper_l_finger', 'l_gripper_r_finger']
```
## conversion:  
>head = head  
>upper_shoulder = S0  
>lower_shoulder = S1  
>upper_elbow = E0  
>lower_elbow = E1  
>upper_forearm = W0  
>lower_forearm = W1  
>wrist = W2  

roboticstoolbox -> ros:
```
[0, 17, 18, 12, 13, 10, 11, 14, 15, 16, 8, 9, 3, 4, 1, 2, 5, 6, 7]
```

ros -> roboticstoolbox:
```
[0, 14, 15, 12, 13, 16, 17, 18, 10, 11, 5, 6, 3, 4, 7, 8, 9, 1, 2]
```

## the indices need to be matched similarly for the position-based servoing code


### name: 
  - head_nod
  - head_pan
  - left_e0
  - left_e1
  - left_s0
  - left_s1
  - left_w0
  - left_w1
  - left_w2
  - right_e0
  - right_e1
  - right_s0
  - right_s1
  - right_w0
  - right_w1
  - right_w2
  - torso_t0

### rtb:
```
0 'head'
1 'right_upper_shoulder'  r_s0
2 'right_lower_shoulder'  r_s1
3 'right_upper_elbow'     r_e0
4 'right_lower_elbow'     r_e1
5 'right_upper_forearm'   r_w0
6 'right_lower_forearm'   r_w1
7 'right_wrist'           r_w2
8 'r_gripper_l_finger'
9 'r_gripper_r_finger'
10 'left_upper_shoulder'  l_s0
11 'left_lower_shoulder'  l_s1
12 'left_upper_elbow'     l_e0
13 'left_lower_elbow'     l_e1
14 'left_upper_forearm'   l_w0
15 'left_lower_forearm'   l_w1
16 'left_wrist'           l_w2
17 'l_gripper_l_finger'
18 'l_gripper_r_finger'
```

## TRY:
Baxter API
brute force setting

## problem is?:
- rtb associates each joint with a link  
- even though head link actually has two joints (head_pan, head_nod) this is not accounted for  
- torso also has no joint, but shows as joint in ros  
- also the gripper links/joints are specified together, while for ros it is somehow less connected (do not show together in topic /robot/joint_states or /robot/joint_names)


PYTHONPATH=/opt/baxter/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages:/home/scott/.local/lib/python3.9/site-packages/