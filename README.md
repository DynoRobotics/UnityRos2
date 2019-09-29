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

How can I try this out?
-----------------------
There are some prebuilt binaries available for this project that can make it easier to get started. If you want not use custom ROS2 messages from Unity you will need to build the project from source in order to generate c# libraries for them.

You need to install the standard ROS2 Dashing dependencies for your operating system of choice https://index.ros.org/doc/ros2/Installation/
If you are building this project from source you will probably need to install the "from source" dependencies for ROS2 as well.

To build from source you will also need .Net Core https://www.microsoft.com/net/learn/get-started. On Windows 10 I recommend installing .NET via Visual Studio 2017.

Install Unity
https://forum.unity.com/threads/unity-hub-v-1-5-0-is-now-available.627847/

*NOTE: Tested with Unity 2019.2.6f1*

Tutorials
---------
https://unity-ros2.readthedocs.io

Installing
----------

The following steps describe how to install the example Unity/ROS2 project on Linux and Windows 10:

Windows 10
----------

Installing dependencies
-----------------------
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

### Option 1: Using the prebuilt project
Download the latest unityPackage for Windows 10 from [releases](https://github.com/DynoRobotics/unity_ros2/releases)

```
call \dev\standalone_unity_ws\install\local_setup.bat

```
### Option 2: Building from source
```
md \dev\ros2_unity_ws\src
cd \dev\ros2_unity_ws
curl -sk https://github.com/DynoRobotics/unity_ros2/raw/dashing_minimal/ros2_unity_standalone.repos -o ros2_unity.repos
vcs import src < ros2_unity_win10.repos
colcon build
call install\setup.bat
```

Now you can open this repository as a project in Unity and run the example scene that has a publisher and subscription. Make sure you open Unity in a terminal with everything sourced, or to add inte bin and lib folders from both workspaces to PATH.
The repository sould be located at dev\ros2_unity_ws\src\dotnet\unity_ros2 if you followed the instructions above.
Make sure you open Unity in a shell where you have sourced setup.bash so that the linker can find the C libraries.



Example Applications
--------------------
[![Sweepbot](https://img.youtube.com/vi/eMKbbEQhBTg/0.jpg)](https://www.youtube.com/watch?v=nggGs9ZIdlk)
[![Package DropOff](https://img.youtube.com/vi/2is7kwPeydA/0.jpg)](https://www.youtube.com/watch?v=lptKRANOfCY)
