Tutorial: Turtlebot3 Demo Scene
================================

.. _unity_ros2: https://github.com/DynoRobotics/unity_ros2

.. figure:: /_static/turtlebot3-navigation.gif
   :width: 100%
   :align: center
   :figclass: align-centered

.. note::
    Prerequisites -- The following is required for this tutorial

    * Unity3d 2018.3.6f1 (other versions could also work)
    * Docker and docker-compose
    * Git
    * Curl (or wget for Linux)
    * unity_ros2 for your platform. Follow the instructions at `unity_ros2`_.

The goal of this tutorial is to get the turtlebot3 sample scene up and running and to explore some of the features in unity_ros2.

The tutorial assumes that you have installed `unity_ros2`_ by downloading and extracting the prebuilt to :code:`C:\dev\standalone_unity_ws`. It also assumes that you are running everything on Windows 10. Modify terminal commands as appropriate if you are using Linux or a different installation path.

Running the Unity project
^^^^^^^^^^^^^^^^^^^^^^^^^

Before you open Unity you need to source the ROS2 workspace. The reason for this is that the linker has to be able to find the ROS2 libraries.

.. _ros2_dotnet: https://github.com/esteve/ros2_dotnet
.. note::
    It is also possible to add the bin and lib folders to PATH permanently.
    Yet another option is to configure the Visual Studio or Monodevelop solution
    to copy over the DLL or .so files to the build directory for the Unity project.
    The instructions at `ros2_dotnet`_ could be helpful for the latest option.

.. code-block:: console

    call \dev\standalone_unity_ws\install\setup.bat

.. _Unity Hub: https://unity3d.com/get-unity/download

The next step is to start Unity. We need to do this in the same terminal where we sourced the ROS2 workspace.
I recommend using `Unity Hub`_ to allow easy switching between versions of Unity.
Copy the shortcut to UnityHub to ``C:\dev\UnityHub.lnk`` before writing the following in the terminal:

.. code-block:: console

    \dev\UnityHub.lnk

.. figure:: /_static/UnityHub.png
   :width: 100%
   :align: center
   :figclass: align-centered

Select `Open` in UnityHub, navigate to the unity_ros2 project in the ``src\dotnet`` directory of the ROS2 workspace and click on `Select Folder`.

.. figure:: /_static/UnityVersions.png
   :width: 100%
   :align: center
   :figclass: align-centered

You will then be prompted to select what version of Unity to open the project with. This tutorial is tested with `2018.3.6f1`. If that version is not available in the dropdown list, you need to install it using the `Installs` tab of UnityHub.

.. figure:: /_static/turtlebot3-scene-select.png
   :width: 100%
   :align: center
   :figclass: align-centered

Double click on `Turtlebot3NavigationDemo` located under ``Assets/Scenes`` in the `Project` window.

.. figure:: /_static/turtlebot3-scene-initial.png
   :width: 100%
   :align: center
   :figclass: align-centered

Unity should now look similar to the image above. (Depending on your layout settings for Unity).

.. figure:: /_static/UnityPlay.png
   :width: 100%
   :align: center
   :figclass: align-centered

Press the `Play` button to start the simulation.

.. figure:: /_static/EmptyConsole.png
   :width: 100%
   :align: center
   :figclass: align-centered

If everything is working, there should be no error messages in the Console.

.. figure:: /_static/turtlebot3-echo.gif
   :width: 100%
   :align: center
   :figclass: align-centered

To teleop the turtlebot3 with you keyboard (wasd or arrows), first set focus to the `Game` window.
If you want to, you can verify that the ROS2 communication is working by writing ``ros2 topic echo /cmd_vel`` in your original terminal window.

Running navigation2 for turtlebot3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The easiest way (in my opinion...) to test the navigation2 stack with turtlebot3 and unity_ros2 is to use our precompiled docker image from DockerHub.
Start by downloading the docker-compose file:

.. code-block:: console

    md \dev\turtlebot3_docker
    cd \dev\turtlebot3_docker
    curl -sk https://raw.githubusercontent.com/DynoRobotics/turtlebot3_unity/master/docker-compose.yaml -o docker-compose.yaml
    docker-compose up

.. figure:: /_static/turtlebot3-compose-pull.gif
   :width: 100%
   :align: center
   :figclass: align-centered

The first time you run ``docker-compose up`` the image should be pulled automatically.

.. figure:: /_static/turtlebot3-compose-up.png
   :width: 100%
   :align: center
   :figclass: align-centered

You should eventually see something like the image above in the terminal window.

.. note::
    Sending navigation goals to ``/move_base_simple/goal`` before you see the line ``[]: Looping due to no latching at the moment`` could prevent navigation form working at the moment of writing. Could be fixed by the time you read this...

Sending navigation goals and missions from Unity
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: /_static/turtlebot3-send-nav-goal.png
   :width: 100%
   :align: center
   :figclass: align-centered

Now that everything is up and running, we should be able to send some navigation goals to the robot.
Start by selecting a goal in the `Hierarch` window in Unity, then press `Send Navigation Goal` in the `Inspector` window.

.. figure:: /_static/turtlebot3-nav-success.gif
   :width: 100%
   :align: center
   :figclass: align-centered

.. note::
    If no path shows up on Window 10, you may need to disable Window Defender or figure out the correct settings.
    For some reason "large" incoming messages like the `Path` messages can be blocked for some reason.

.. figure:: /_static/mission-planner.png
   :width: 100%
   :align: center
   :figclass: align-centered

The example scene also has an interface for the navigation2 mission planner.
You can add a sequence of navigation goals and execute them in order using this.

.. figure:: /_static/mission-xml.png
   :width: 100%
   :align: center
   :figclass: align-centered

The xml string representing the plan will also be written to the `Console` window, in case you want to copy it in your code or do something similar.

That's all for this tutorial. Good luck and have fun!
