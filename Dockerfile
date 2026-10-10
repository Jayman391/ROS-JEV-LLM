# ROS 2 Humble + Gazebo Fortress on Ubuntu 22.04 (jammy).
# Humble targets jammy, so this image is jammy even though the host is macOS.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Base tools and locale
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl gnupg lsb-release ca-certificates software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 apt repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list

# Gazebo (OSRF) apt repo, provides Fortress, the Gazebo version paired with Humble
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# ROS 2 desktop (rviz2, demos), ROS<->Gazebo bridge, and build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-ros-gz \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        git \
        vim \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# Source ROS in every interactive shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Virtual display for GUI apps (Gazebo GUI, rviz2). XQuartz can't provide the
# OpenGL visuals Gazebo needs, so the GUI runs on Xvfb and is viewed via noVNC.
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb x11vnc novnc websockify libgl1-mesa-dri mesa-utils \
        xdotool openssl \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh docker/gui-fullscreen /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/gui-fullscreen

# jev_ai live-backend SDKs (anthropic, typesafe) with pinned versions
COPY src/jev_ai/requirements.txt /tmp/jev_ai-requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/jev_ai-requirements.txt
# anyio's pytest plugin (pulled in by the SDKs) breaks Jammy's pytest 6
ENV PYTEST_ADDOPTS="-p no:anyio"

# Same environment as setup.bash, so `docker compose exec` commands can run
# ROS tools without a login shell
ENV AMENT_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib \
    PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages \
    PATH=/opt/ros/humble/bin:/usr/local/bin:/usr/bin:/bin \
    DISPLAY=:99 \
    VNC_PASSWORD=password

WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]
