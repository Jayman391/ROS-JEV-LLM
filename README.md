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

## Run a shell in the container

```bash
xhost +localhost          # allow XQuartz clients (run on the Mac, once per login)
docker compose run --rm ros2
```

The repo is mounted at `/ws/ROS-JEV` inside the container, so edits on the Mac show up there immediately. ROS is sourced automatically in every shell.

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
ign gazebo -s -r shapes.sdf
```
`-s` runs the server only and `-r` starts the simulation immediately. Press `Ctrl+C` to stop. Gazebo Fortress uses the `ign` command, not `gz`.

**Gazebo GUI** (needs XQuartz):
```bash
ign gazebo shapes.sdf
```

**ROS ↔ Gazebo bridge:** with the server running in one shell, check the topics it exposes from another:
```bash
ign topic -l
ros2 run ros_gz_bridge parameter_bridge --help
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
