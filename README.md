ROS2 for Unity
==============

Build status
------------
[![CircleCI](https://circleci.com/gh/DynoRobotics/unity_ros2/tree/master.svg?style=svg)](https://circleci.com/gh/DynoRobotics/unity_ros2/tree/master)

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for C# specificly targeted at Unity.

Features
--------

The current set of features include:
- Generation of all builtin ROS types, dynamic arrays and nested types.
- Support for publishers and subscriptions
- Tests

What's missing?
---------------

Lots of things!
- Arrays of nested types
- Static Arrays
- Clients and services
- Actions
- Time
- Documentation
- More examples
- Cross-platform support (Windows, Windows IoT Core, UWP) [sould work with minimal modifications, but not tested]
- Automatic test runner for CI (C#)

Sounds great, how can I try this out?
-------------------------------------

First of all install the standard ROS2 Crystal Clemmys dependencies for your operating system of choice https://index.ros.org/doc/ros2/Installation/

Next make sure you've installed Mono https://www.mono-project.com/ or .Net Core https://www.microsoft.com/net/learn/get-started (not tested, use master branch of dotnet_cmake_module if you do)

Install Unity
https://forum.unity.com/threads/unity-hub-v-1-0-0-is-now-available.555547/

The following steps show how to build the example Unity/ROS2 project on Linux:

Linux
-----

```
mkdir -p ~/ros2_unity_ws/src
cd ~/ros2_unity_ws
wget https://github.com/DynoRobotics/unity_ros2/raw/master/ros2_unity.repos
vcs import src < ros2_unity.repos
colcon build
source install/setup.bash
```

Now you can open this repository as a project in Unity and run the example scene that has a publisher and subscription.
The repository sould be located at ~/ros2_unity_ws/src/dotnet/unity_ros2 if you followed the instructions above.
Make sure you open Unity in a shell where you have sourced setup.bash so that the linker can find the C libraries.
