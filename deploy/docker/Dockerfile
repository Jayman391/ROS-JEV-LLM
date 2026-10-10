# jev_ai runtime image (FRAMEWORK §13): ros-base + the workspace, one image for every node.
#
#   docker build -f deploy/docker/Dockerfile -t jev-ai:humble .
#   docker build -f deploy/docker/Dockerfile --build-arg ROS_DISTRO=jazzy -t jev-ai:jazzy .
#
# The dev image (repo-root Dockerfile: desktop, Gazebo, noVNC) is for working in; this one is
# what compose and k8s run. Keys are never baked in: env vars or a mounted file at run time.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
SHELL ["/bin/bash", "-c"]
WORKDIR /ws

# System dependencies from the package manifests only, so code edits keep this layer cached.
COPY src/jev_ai/package.xml src/jev_ai/
COPY src/jev_ai_msgs/package.xml src/jev_ai_msgs/
COPY src/jev_ai_servers/package.xml src/jev_ai_servers/
COPY src/jev_ai_bringup/package.xml src/jev_ai_bringup/
COPY src/jev_ai_cpp/package.xml src/jev_ai_cpp/
COPY src/jev_ai_bt/package.xml src/jev_ai_bt/
COPY src/jev_ai_examples/package.xml src/jev_ai_examples/
COPY src/ros2jev/package.xml src/ros2jev/
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
        --skip-keys python3-pytest \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Live-backend SDKs, pinned. Jazzy (noble, PEP 668) needs --break-system-packages; inside a
# single-purpose image that is the intended use.
COPY src/jev_ai/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir $( [ "${ROS_DISTRO}" = humble ] || echo --break-system-packages ) \
        -r /tmp/requirements.txt
ENV PYTEST_ADDOPTS="-p no:anyio"

COPY src/ src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

COPY deploy/docker/entrypoint.sh /entrypoint.sh
# turtlesim (A9 example) has no display in a pod; records go to a volume
ENV QT_QPA_PLATFORM=offscreen \
    JEV_AI_DASHBOARD_HOST=0.0.0.0
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "jev", "status"]
