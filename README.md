# ROS-JEV

ROS 2 Humble + Gazebo Fortress, running in a Docker container on macOS (Apple Silicon).

Humble targets Ubuntu 22.04, and Humble's official Gazebo pairing is Fortress. macOS can't run either natively, so the whole stack runs in an Ubuntu 22.04 container under Docker Desktop.

GUI apps (the Gazebo GUI and `rviz2`) run on a virtual display inside the container and are viewed in your browser through noVNC. XQuartz isn't needed.

## Prerequisites

- **Docker Desktop** (Apple Silicon build). Start it and wait for the whale icon to settle.

## Build

From the repo root:

```bash
docker compose build
```

The first build downloads `ros-humble-desktop` and Gazebo Fortress, so it takes a while and needs roughly 10 GB of free disk.

## Start the container

```bash
docker compose up -d                 # start the container in the background
docker compose exec ros2 bash        # open a shell inside it
```

The repo is mounted at `/ws/ROS-JEV` inside the container, so edits on the Mac show up there immediately. ROS is sourced in every `bash` shell. Stop it with `docker compose down`.

Open a separate `docker compose exec ros2 bash` shell for each process (Gazebo, the bridge, and ROS tools). They all share the same container, so they can see each other's topics.

## Run Gazebo with the ROS connection

**Gazebo server + GUI:**
```bash
docker compose exec ros2 ign gazebo diff_drive.sdf
```
Then open **http://localhost:6080/vnc.html** in your browser and click **Connect**. The Gazebo window appears there. Use `-s -r` (`ign gazebo -s -r diff_drive.sdf`) for a headless server with no GUI.

**ROS 2 bridge** (second shell):
```bash
docker compose exec ros2 ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
  '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
```
Each bridge argument is `ROS_topic@ROS_type[Gazebo_type`. The `[` means Gazebo → ROS, and `@` separates the ROS type from the Gazebo type. Quote the arguments in zsh, because `[` is a glob character.

**Read ROS messages** (third shell):
```bash
docker compose exec ros2 ros2 topic list
docker compose exec ros2 ros2 topic echo /clock --once
docker compose exec ros2 ros2 topic echo /model/vehicle_blue/odometry --once
```

**rviz2** (view in the same noVNC page):
```bash
docker compose exec ros2 rviz2
```

**Gazebo topics without ROS:**
```bash
docker compose exec ros2 ign topic -l
```

## Verify the install

**ROS 2 talker/listener:**
```bash
docker compose exec ros2 ros2 run demo_nodes_cpp talker &
docker compose exec ros2 ros2 run demo_nodes_cpp listener
```
You should see `I heard: [Hello World: N]` lines.

**OpenGL on the virtual display:**
```bash
docker compose exec ros2 glxinfo -B | grep renderer
```
Should print `llvmpipe` (software rendering).

## Troubleshooting

- **noVNC page is blank or won't connect:** Check the container is up (`docker compose ps`) and that the GUI process started. The virtual display is created when the container starts, so run `docker compose restart` if it's missing.
- **Gazebo GUI is slow:** Software rendering is slower than a GPU but works. Use `-s -r` for headless mode, or make the window smaller.
- **Stale image after changing the Dockerfile:** `docker compose build --no-cache`.
- **Port 6080 already in use:** Stop the other process using it, or change the left side of `"127.0.0.1:6080:6080"` in `docker-compose.yml`.

## Files

- `Dockerfile`: Ubuntu 22.04 with ROS 2 Humble desktop, `ros-humble-ros-gz`, Gazebo Fortress, and the virtual display stack.
- `docker/entrypoint.sh`: starts Xvfb, x11vnc, and noVNC when the container starts.
- `docker-compose.yml`: builds the image, mounts the repo, and publishes the noVNC port on localhost.
- `.dockerignore`: keeps `.git`, `.venv`, and caches out of the build context.
