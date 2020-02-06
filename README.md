ROS2 for Unity
==============

![Turtebot3 Navigation2](https://i.gyazo.com/98d3d43aae3877593ecaefe4e5ba9a44.gif)

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for C# specificly targeted at Unity.

Platform support
----------------
This project curretly supports Ubuntu 18.04. Windows 10 was supported in an earlier versions of this project, but is not yet available in the standalone version. It should be possible to get it working on other systems as well.

How can I try this out?
-----------------------
Clone this repo and open the Unity Project using the Unity Hub.
We plan to make this be available as a .unitypackage and in the Unity Asset Store as soon as possible.

Installing Unity
----------------
Install Unity through Unity Hub
https://forum.unity.com/threads/unity-hub-v-1-6-0-is-now-available.640792
(the download link changes sometimes, so look at the newest posts)

After downloading Unity Hub install Unity Editor. Follow this instruction https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html When choosing a version, choose the latest stable one. During installation, wait until the blue line is complete.

Building custom interfaces/messages
-----------------------------------
Clone https://github.com/samiamlabs/ros2_dotnet (cyclone branch) and run the script `create_unity_plugin.bash` to build/generate a "Plugins" folder for Unity from source.
This script requires that you have docker installed on your system permissions setup to run it without sudo. See https://dyno-system-config.readthedocs.io/en/latest/docker.html

If you want some custom messages included in the prebuilt binaries, you are welcome to submit a pull request with a ROS2 package containing the interfaces/messages to https://github.com/DynoRobotics/rcldotnet_custom_interfaces

To build custom messages yourself, you will need to add the packages containing them to https://github.com/samiamlabs/ros2_dotnet/blob/cyclone/ros2_dotnet_dashing.repos and https://github.com/samiamlabs/ros2_dotnet/blob/cyclone/rcldotnet_utils/rcldotnet_utils/create_unity_plugin.py before running the build script. Note that all the .so files need to be set to preload at startup in Unity. (Can be selected in a checkbox in the "inspector" window)

Features
--------

The current set of features include:
- Generation of all builtin ROS types, dynamic arrays and nested types.
- Support for publishers and subscriptions
- Quality of service profiles
- Time (limited support)
- Laser scan
- Tf publisher
- Pose publisher
- Odometry (ground truth, no noise yet)
- Base controller
- Basic Navigation2 support
- Sending nav goals
- Path visualizer
- Keyboard teleop
- Tests
- and more...


What's missing?
---------------
- More examples
- Better cross-platform support (Windows IoT Core, UWP, Mac) [should work with minimal modifications, but not tested]
- Automatic test runner for CI (C#)
- Costmap visualizer
- Camera sensor
- Camera viewer
- 3d sensor
- Pointcloud visualizer
- Video visualiszer
- ros2_control support
- Hololens support

Example Applications
--------------------
[![Sweepbot](https://img.youtube.com/vi/eMKbbEQhBTg/0.jpg)](https://www.youtube.com/watch?v=nggGs9ZIdlk)
[![Package DropOff](https://img.youtube.com/vi/2is7kwPeydA/0.jpg)](https://www.youtube.com/watch?v=lptKRANOfCY)
