#!/bin/bash
# Start a virtual X display with software OpenGL, plus a VNC server and a
# browser-based noVNC client, then run the container command.
#
# Why: XQuartz on macOS does not provide the GLX visuals Gazebo needs, so GUI
# apps fail with "No matching fbConfigs or visuals found". Xvfb renders in
# software (Mesa llvmpipe) inside the container, and noVNC shows it in a browser
# at http://localhost:6080/vnc.html.

set -e

Xvfb :99 -screen 0 1600x900x24 +extension GLX -ac -nolisten tcp &
export DISPLAY=:99

# Give Xvfb a moment to create its socket before VNC attaches
sleep 2

x11vnc -display :99 -forever -shared -nopw -localhost -rfbport 5900 -quiet &

websockify --web /usr/share/novnc 6080 localhost:5900 &

exec "$@"
