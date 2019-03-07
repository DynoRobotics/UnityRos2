Unity for ROS2
==============

![Turtebot3 Navigation2](https://i.gyazo.com/98d3d43aae3877593ecaefe4e5ba9a44.gif)

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for C# specificly targeted at Unity.

Platform support
----------------
This project curretly supports Windows 10 and Ubuntu 16.04. It should be possible to get it working on other systems as well.

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
- Tf subscriber/visualizer
- Odometry (ground truth, no noise yet)
- Base controller
- Joint controller interface (only position so far)
- Joint state publiser
- Distance sensor
- Navigation2 support
- Sending nav goals
- Sending navigation missions
- Path visualizer
- Keyboard teleop
- Tests
- and more...


What's missing?
---------------

Plenty of stuff!
- Clients and services
- Setting individual element of array in message (need to assign complete array atm.)
- Actions
- Documentation
- More examples
- Better cross-platform support (Windows IoT Core, UWP, Mac) [should work with minimal modifications, but not tested]
- Automatic test runner for CI (C#)
- Costmap visualizer
- Camera sensor
- 3d sensor
- Pointcloud visualizer
- Video visualiszer
- Hololens support

How can I try this out?
-------------------------------------

First of all install the standard ROS2 Crystal Clemmys dependencies for your operating system of choice https://index.ros.org/doc/ros2/Installation/

Next make sure you've installed .Net Core https://www.microsoft.com/net/learn/get-started. On Windows 10 I recommend installing .NET via Visual Studio 2017.

Install Unity
https://forum.unity.com/threads/unity-hub-v-1-5-0-is-now-available.627847/

*NOTE: Tested with Unity 2018.3.6f1*

The following steps show how to build the example Unity/ROS2 project on Linux:

Linux
-----

```
mkdir -p ~/ros2_unity_ws/src
cd ~/ros2_unity_ws
wget https://github.com/DynoRobotics/unity_ros2/raw/master/ros2_unity.repos
vcs import src < ros2_unity.repos
colcon build --merge-install
source install/setup.bash
```

Now you can open this repository as a project in Unity and run the example scene that has a publisher and subscription.
The repository sould be located at ~/ros2_unity_ws/src/dotnet/unity_ros2 if you followed the instructions above.
Make sure you open Unity in a shell where you have sourced setup.bash so that the linker can find the C libraries.

Windows 10
----------

```
md \dev\ros2_unity_ws\src
cd \dev\ros2_unity_ws
curl -sk https://github.com/DynoRobotics/unity_ros2/raw/master/ros2_unity.repos -o ros2_unity.repos
vcs import src < ros2_unity_win10.repos
call \dev\ros2\install\local_setup.bat
colcon build --merge-install
call install\setup.bash
```

Now you can open this repository as a project in Unity and run the example scene that has a publisher and subscription. Make sure you open Unity in a terminal with everything sourced, or to add inte bin and lib folders from both workspaces to PATH.
The repository sould be located at dev\ros2_unity_ws\src\dotnet\unity_ros2 if you followed the instructions above.
Make sure you open Unity in a shell where you have sourced setup.bash so that the linker can find the C libraries.

Tutorials
---------
https://unity-ros2.readthedocs.io

Example Applications
--------------------
[![Sweepbot](https://img.youtube.com/vi/eMKbbEQhBTg/0.jpg)](https://www.youtube.com/watch?v=eMKbbEQhBTg)
[![Package DropOff](https://img.youtube.com/vi/2is7kwPeydA/0.jpg)](https://www.youtube.com/watch?v=2is7kwPeydA)

