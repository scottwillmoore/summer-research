# Oculus Quest 2

## Overview

To develop in Unity I would read both the [Oculus documentation](https://developer.oculus.com/documentation/unity/) and the [Unity documentation](https://docs.unity3d.com/Manual/VROverview.html) for VR development. The landscape of VR development tools appears to have evolved quite drastically over the last several years, therefore I would make sure to check the dates of articles on the internet and double check that the advice is still applicable.

## Connection

You can read more about it on the internet, and in the Oculus documentation, but the Oculus Quest (1 and 2) platform is a standalone device derived from Android. Applications run directly on the hardware, but Oculus provides Oculus Link to allow your PC to render and run the application. You can also enable and use Oculus Air Link which allows you to stream the PC application over WiFi, however I have found it not quite as stable. I would recommend using the link cable (which should be plugged into a USB 3.1+ port) for development, etc.

## Setup

My advice would be to follow the "Get Started" section of the Unity Oculus documentation as closely as possible. Keep in mind that this approach used to use an Oculus-specific SDK which is now deprecated, however it has been updated such that it now used the OpenXR backend, an agnostic SDK which is adopted by all VR manufacturers. You still have the ability to use the Oculus SDK backend, which does provide a few features not supported by the OpenXR standard.

These steps will show you how to build and run the Android application, however it is slow to build and upload to the device so I would instead recommend this approach instead: Install the [Oculus Link software](https://www.oculus.com/setup/) on your computer and setup a connection to the Oculus Quest 2 (wired or wireless). In Unity, in your project settings enable Oculus as the OpenXR backend for desktop. You should now be able to run the preview in the Unity editor.

It is up to you to decide whether you want to deploy the application for desktop or android (runs directly on the Oculus Quest 2).

## Integration

The Oculus Integration package for Unity is actually not required when developing for OpenXR, but it does import some useful assets into your project. The "Core Development Blocks" section of the Unity Oculus documentation has more information. Some of the folders such as the "Avatar" folder I don't think are well supported on the OpenXR backend.

## Conclusion

There is probably a lot more little issues that you might encounter, such as objects rendering in only one eye. Let me know if you have any other troubles, I can't quite remember all the issues I had, but I should still remember the solutions. You are more than welcome to ask me any questions. I am not an expert at Unity of the Oculus Quest 2 by any means, but I may be able to help.
