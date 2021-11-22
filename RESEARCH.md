# Overview

Random notes...

Deploy to Oculus with the Mobile SDK. This will allow the Oculus to be used as a standalone device.

Deploy the control software to the robot. This will allow the robot and Oculus headset to be used without any other devices.

Communication between Windows and ROS will be difficult as Windows is not supported by ROS (Windows does have support for ROS 2, however Baxter requires ROS). MATLAB does support ROS on Windows, which implies that it is possible to build a ROS interface layer on Windows.

Windows is not required, as the Oculus Mobile SDK is used. The Oculus is Android-based (Linux), therefore it may be possible to compile the ROS communication components for it to use.

The Oculus may be able to be developed on Linux. This could simplify development of the communication layer. This is unsupported. [Read here](https://lordsoftech.com/tech/development-for-oculus-quest-is-totally-doable-on-linux/).

# Oculus

## Notes

There is a Mobile SDK and a PC SDK.

Development SDKs are only provided for Windows.

Applications can be developed with Unity and Unreal Engine, or from scratch with the C++ SDKs. The OpenXR API is also supported.

The Mobile experience is powered by a Qualcomm Snapdragon XR2 Platform. It is an Android-based device.

## Tasks

- Get a development device with a GPU

- Setup the development device with Windows 10 or 11

- Install the Oculus SDKs

- Test the headset with an example program

- Decide on a game engine (Unity or Unreal Engine)

- Setup the game engine

- Run an example project for the game engine

- Create a new project

- Experiment with the APIs

  - Render a cube
  - Get and input from the user
  - Render the Baxter meshes

- Ensure Baxter is setup

- Experiment with Baxter to decide on a user-interface

- Establish communication between Baxter and the project

- ...

## References

[Quest 2](https://www.oculus.com/quest-2/)

[Develop for the Quest Platform](https://developer.oculus.com/quest/)

# Baxter

## Notes

Code can be run on a workstation, or directly on the internal CPU.

Rethink Robotics has closed down. Baxter support is no longer provided.

I don't think the Baxter source code is available, however, all the interfaces to the robot are available.

The update files are hosted on an FTP server which requires access. This may or may not be possible due to the state of the company.

SSH access is provided to Baxter, however this should be used with cation, as there may not be an easy way to fix any mistakes.

It is unclear whether SSH access provides full control over the robot, or just to the on-robot development environment.

The latest version released only supports Ubuntu 14.04 with ROS Indigo.

It may be simpler to use a Docker-based development environment, which may simply the process due to the old, outdated dependencies.

What Wi-Fi network can we connect to the robot.

## Tasks

- Follow the Baxter Setup documentation

- Ensure Baxter hardware is setup

  - Check hardware is complete

  - Power on Baxter

  - Update Baxter (FTP access is required)

- Setup Baxter workstation environment

  - Setup ROS

  - Install the Baxter dependencies (SDK, etc)

- Run diagnostics

- Run an example program

- Experiment with Docker development environment

- Work on communication link with game engine

- ...

## References

[Rethink Robotics](https://www.rethinkrobotics.com/?utm_source=robots.ieee.org)

[Baxter Documentation](https://sdk.rethinkrobotics.com/wiki/Home)

[Baxter GitHub](https://github.com/RethinkRobotics?q=baxter&type=&language=&sort=)
