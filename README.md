# ROS-JEV

ROS 2 Humble + Gazebo Fortress, running in a Docker container on macOS (Apple Silicon).

Humble targets Ubuntu 22.04, and Humble's official Gazebo pairing is Fortress. macOS can't run either natively, so the whole stack runs in an Ubuntu 22.04 container under Docker Desktop.

## Prerequisites

1. **Docker Desktop** (Apple Silicon build). Start it and wait for the whale icon to settle.
2. **XQuartz** (only for GUI apps like `rviz2` and the Gazebo GUI):
   ```bash
   brew install --cask xquartz
   ```
   Log out and back in after the first install. Then open XQuartz and enable **Settings → Security → "Allow connections from network clients"**, and restart XQuartz.

## Build

From the repo root:

```bash
docker compose build
```

The first build downloads `ros-humble-desktop` and Gazebo Fortress, so it takes a while and needs roughly 10 GB of free disk.

## Start the container

```bash
xhost +localhost          # allow XQuartz clients (run on the Mac, once per login)
docker compose up -d      # start the container in the background
docker compose exec ros2 bash   # open a shell inside it
```

The repo is mounted at `/ws/ROS-JEV` inside the container, so edits on the Mac show up there immediately. ROS is sourced in every `bash` shell. Stop it with `docker compose down`.

Open a separate `docker compose exec ros2 bash` shell for each process (Gazebo, the bridge, and ROS tools). They all share the same container, so they can see each other's topics.

## Verify the install

Run these inside the container.

**ROS 2 talker/listener:**
```bash
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```
You should see `I heard: [Hello World: N]` lines.

**Gazebo (headless, no GUI):**
```bash
ign gazebo -s -r diff_drive.sdf
```
`-s` runs the server only and `-r` starts the simulation immediately. Press `Ctrl+C` to stop. Gazebo Fortress uses the `ign` command, not `gz`. `diff_drive.sdf` ships with Fortress and has two driveable vehicles.

**Gazebo GUI** (needs XQuartz):
```bash
ign gazebo diff_drive.sdf
```

**Connect Gazebo to ROS 2:** start the server in one shell, then the bridge in a second shell. Both run inside the same container:
```bash
# Mac terminal 1
docker compose exec ros2 bash
ign gazebo -s -r diff_drive.sdf

# Mac terminal 2
docker compose exec ros2 bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' \
  '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
```
Each bridge argument is `ROS_topic@ROS_type[Gazebo_type`. The `[` means Gazebo → ROS, and `@` separates the ROS type from the Gazebo type. Quote the arguments in zsh, because `[` is a glob character.

Then, in a third shell, read ROS messages:
```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /model/vehicle_blue/odometry --once
```

To list Gazebo's own topics without ROS:
```bash
ign topic -l
```

**rviz2** (needs XQuartz):
```bash
rviz2
```

## Troubleshooting

- **`cannot open display` or no windows:** XQuartz isn't running, network clients aren't allowed, or `xhost +localhost` wasn't run. Restart XQuartz and rerun `xhost +localhost`.
- **Gazebo GUI is slow or crashes:** Use the headless `-s` mode. The GUI in a VM uses software rendering (`LIBGL_ALWAYS_SOFTWARE=1` is set in `docker-compose.yml`), which is slow but works.
- **Stale image after changing the Dockerfile:** `docker compose build --no-cache`.

## Files

- `Dockerfile`: Ubuntu 22.04 with ROS 2 Humble desktop, `ros-humble-ros-gz`, and Gazebo Fortress.
- `docker-compose.yml`: builds the image, mounts the repo, and sets the display variables.
- `.dockerignore`: keeps `.git`, `.venv`, and caches out of the build context.
