#!/bin/bash
# Start a virtual X display with software OpenGL, plus a password-protected VNC
# server and a browser-based noVNC client, then run the container command.
#
# Why: XQuartz on macOS does not provide the GLX visuals Gazebo needs, so GUI
# apps fail with "No matching fbConfigs or visuals found". Xvfb renders in
# software (Mesa llvmpipe) inside the container, and noVNC shows it in a browser
# at http://localhost:6080/vnc.html.
#
# The VNC password comes from VNC_PASSWORD if set. Otherwise a random one is
# generated for this container start and printed to the container log.
#
# No window manager runs here on purpose: one captures mouse drags for window
# moves and resizes, which blocks clicks inside Gazebo. Use gui-fullscreen to
# size windows instead.

set -e

PASSFILE=/tmp/vnc.passwd
if [ -n "$VNC_PASSWORD" ]; then
    VNC_PW="$VNC_PASSWORD"
else
    VNC_PW=$(openssl rand -hex 8)
    echo "noVNC password for this session: $VNC_PW"
fi
# x11vnc only reads passwords from a file; keep it private to this user
(umask 077 && x11vnc -storepasswd "$VNC_PW" "$PASSFILE" >/dev/null)

Xvfb :99 -screen 0 1600x900x24 +extension GLX -ac -nolisten tcp &
export DISPLAY=:99

# Give Xvfb a moment to create its socket before VNC attaches
sleep 2

x11vnc -display :99 -forever -shared -rfbauth "$PASSFILE" -localhost -rfbport 5900 -quiet &

websockify --web /usr/share/novnc 6080 localhost:5900 &

exec "$@"
