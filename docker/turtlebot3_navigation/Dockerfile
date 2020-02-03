FROM dynorobotics/balena-amd64-node-ros2:dashing

ENV DEPENDENCIES_WS /opt/dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src

WORKDIR $DEPENDENCIES_WS

COPY ros2_dependencies.repos $DEPENDENCIES_WS/
RUN vcs import src < ros2_dependencies.repos
# RUN vcs import src < src/navigation2/tools/ros2_dependencies.repos

# Remove this so that we don't need to build gazebo_ros_pkgs
RUN rm -r src/navigation2/nav2_system_tests

# install dependencies
RUN . $ROS2_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

RUN . $ROS2_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args   \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

ENV APP_WS /opt/app_ws
RUN mkdir -p $APP_WS/src

WORKDIR $APP_WS

RUN . $DEPENDENCIES_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args   \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

COPY turtlebot3_unity src/turtlebot3_unity
COPY turtlebot3_unity_bringup src/turtlebot3_unity_bringup
COPY turtlebot3_description src/turtlebot3_description

RUN . $DEPENDENCIES_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args   \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

RUN echo "source $APP_WS/install/setup.bash" >> $HOME/.bashrc

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
