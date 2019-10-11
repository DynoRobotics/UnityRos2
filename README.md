Unity for ROS2
==============

![Turtebot3 Navigation2](https://i.gyazo.com/98d3d43aae3877593ecaefe4e5ba9a44.gif)

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for C# specificly targeted at Unity.

Platform support
----------------
This project curretly supports Windows 10 and Ubuntu 18.04. It should be possible to get it working on other systems as well.

How can I try this out?
-----------------------
There are some prebuilt binaries available for this project that can make it easier to get started. If you want not use custom ROS2 messages from Unity you will need to build the project from source in order to generate C# libraries for them. (see https://github.com/samiamlabs/ros2_dotnet)


Install Unity
https://forum.unity.com/threads/unity-hub-v-1-5-0-is-now-available.627847/

*NOTE: Tested with Unity 2019.2.6f1*

Installing
----------

The following steps describe how to install the example Unity/ROS2 project on Linux and Windows 10:

Windows 10
----------

Installing dependencies
-----------------------
Install the standard ROS2 Dashing dependencies for ROS2 https://index.ros.org/doc/ros2/Installation/

Chocolatey is a package manager for Windows, install it by following their installation instructions:

Install Visual Studio 2019

https://chocolatey.org/
choco install -y python
choco install -y cmake
python -m pip install -U catkin_pkg empy lark-parser opencv-python pyparsing pyyaml setuptools

Please download these packages from [this](https://github.com/ros2/choco-packages/releases/latest) GitHub repository.

Run in terminal as admin:
choco install -y -s <PATH\TO\DOWNLOADS\> asio eigen tinyxml-usestl tinyxml2 log4cxx

Download an OpenSSL installer from [this page](https://slproweb.com/products/Win32OpenSSL.html).Scroll to the bottom of the page and download Win64 OpenSSL v1.0.2. Don’t download the Win32 or Light versions.
Add C:\OpenSSL-Win64\bin\ to PATH under System variables.

Ubuntu 18.04
------------
You need to install the standard ROS2 Dashing dependencies for ROS2 https://index.ros.org/doc/ros2/Installation/

It is probably easiest to follow https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Binary/.

You don't actually need to install the ROS2 binaries, just the dependencies. (The ROS2 binaries are already packaged with the Unity project as Plugins). Installing the ROS2 binaries and running som examples is a good way to make sure that the dependencies are installd correctly though :)

### Option 1: Using the prebuilt unitypackage
Download the latest unityPackage for Windows 10 from [releases](https://github.com/DynoRobotics/unity_ros2/releases)

### Option 2: Clone this repo
Just clone this repository to your local computer and open it accodring to the instructions below.

Running
-------
The ROS2 integration in UnityRos2 requires that some enviroment variables are set. (This has to do with the finding and linking of `.so` and `.dll` binaries to C# code)

A script called `start_editor.py` (requires python3) sets these environment variables and starts the Editor. This is only way to start the Editor with ROS2 support at the moment. Opening the project through Unity Hub is not supported yet.

Tutorials
---------
https://unity-ros2.readthedocs.io

Features
--------

The current set of features include:
- Generation of all builtin ROS types, dynamic arrays and nested types.
- Support for publishers and subscriptions
- Quality of service profiles
- Time (limited support)
- Importing robots from urdf
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

Lots!
- Sending navigation missions
- Distance sensor
- Joint controller interface (only position so far)
- Joint state publiser
- Tf subscriber/visualizer
- Clients and services
- Setting individual element of array in message (need to assign complete array atm.)
- Actions
- Documentation
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
