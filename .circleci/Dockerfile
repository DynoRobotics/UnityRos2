FROM dynorobotics/ros2:crystal
ENV LANG en_US.UTF-8

# Mono/C#
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF \
  && echo "deb https://download.mono-project.com/repo/ubuntu stable-bionic main" | tee /etc/apt/sources.list.d/mono-official-stable.list \
  && apt-get update \
  && apt-get install -y \
    apt-transport-https \
    mono-complete \
  && rm -rf /var/likb/apt/lists/*

# ROS2
RUN apt-get update && apt-get install -y \
  python3-colcon-common-extensions \
  python-rosdep \
  python3-vcstool \
  && rm -rf /var/likb/apt/lists/*
