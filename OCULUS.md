# Oculus Quest 2

## Overview

To develop in Unity I would read both the [Oculus documentation](https://developer.oculus.com/documentation/unity/) and the [Unity documentation](https://docs.unity3d.com/Manual/VROverview.html) for VR development. The landscape of VR development tools appears to have evolved quite drastically over the last several years, therefore I would make sure to check the dates of articles on the internet and double check that the advice is still applicable.

You can read more about it on the internet, and in the Oculus documentation, but the Oculus Quest (1 and 2) platform is a standalone device derived from Android. Applications run directly on the hardware, but Oculus provides Oculus Link to allow your PC to render and run the application. You can also enable and use Oculus Air Link which allows you to stream the PC application over WiFi, however I have found it not quite as stable. I would recommend using the link cable, which should be plugged into a USB 3.1+ port.

## Getting started

To setup an Oculus Quest 2 for development.

### Create an Oculus account

It is best to [create an unmerged Oculus developer account](https://developer.oculus.com/sign-up/) with your Monash email address. This does not require it to be linked to a Facebook account. This link may not work if you are logged in with Facebook, so it you may need to open it in an incognito window.

I would suggest that you follow a similar format as the example below when you create your account.

```
User: monash_smoo38
First name: Scott
Last name: Moore
Email address: smoo38@student.monash.edu
```

This account will not have a Facebook account linked, which I believe Oculus may be phasing out in the future... It is not that well supported, or documented, but, it does work for now!

### Join the Monash HRI Group organisation

Once created, send me an email with your username, I can add you to the “Monash HRI Group” (developer) organisation which will allow you to setup applications, test accounts, etc. It will also allow us to publish apps to the Oculus store.

On Unity for some features an application ID is required.

Any test account that is created will come with a mock Facebook account. I believe the test accounts will allow you to test features that require Facebook integration, but may not be needed for our purposes.

### Install the Oculus Developer Hub for Windows

To develop on the device I would highly recommend Windows. This is the only (actually) supported platform. I believe it was possible on a Mac, and is maybe possible on Linux, but it is not really supported.

The [Oculus Developer Hub for Windows](https://developer.oculus.com/downloads/package/oculus-developer-hub-win/) is not well known, but it allows you to setup an Oculus device without the Oculus app for you phone.

Install the Oculus Developer Hub for Windows. This will allow you to manage the device from your computer. It also allows you to enable developer mode without the app installed on your phone.

Login to your unmerged Oculus developer account and connect to the headset.

I don’t know, but it may be better to get me to add your account to the Oculus device on my computer. It may not let you, since my developer account is already setup...

### Install the Oculus Link for Windows

Install [Oculus Link for Windows](https://www.oculus.com/setup/). I believe this is required if you want to quick preview (press the play button) in Unity. However, I don’t think it is actually required in other cases...

Login to your unmerged Oculus developer account and connect to the headset.

## Develop with Unity

My advice would be to follow the ["Get Started" section of the Unity Oculus documentation](https://developer.oculus.com/documentation/unity/unity-gs-overview/) as closely as possible. Keep in mind that this approach used to use an Oculus-specific SDK which is now deprecated, however it has been updated such that it now used the OpenXR backend, an agnostic SDK which is adopted by all VR manufacturers. You still have the ability to use the Oculus SDK backend, which does provide a few features not supported by the OpenXR standard.

These steps will show you how to build and run the Android application, however it is slow to build and upload to the device. Provided you have the Oculus Link software installed in Unity in your project settings you should be able to enable Oculus as the OpenXR backend for desktop. You should now be able to run the preview in the Unity editor.

It is up to you to decide whether you want to deploy the application for desktop or android (runs directly on the Oculus Quest 2).

The Oculus Integration package for Unity is actually not required when developing for OpenXR, but it does import some useful assets into your project. The "Core Development Blocks" section of the Unity Oculus documentation has more information. Some of the folders such as the "Avatar" folder I don't think are well supported on the OpenXR backend.

## Conclusion

There is probably a lot more little issues that you might encounter, such as objects rendering in only one eye. Let me know if you have any other troubles, I can't quite remember all the issues I had, but I should still remember the solutions. You are more than welcome to ask me any questions. I am not an expert at Unity of the Oculus Quest 2 by any means, but I may be able to help.
