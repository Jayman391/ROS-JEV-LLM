# Dev/test image: the base ROS image plus the pinned SDKs used by jev_ai's live backends.
#   docker build -f docker/dev.Dockerfile -t ros-jev:dev src/jev_ai
FROM ros-jev:humble
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt
# anyio (pulled in by httpx) ships a pytest plugin that needs pytest >= 7; Humble has 6.2.
ENV PYTEST_ADDOPTS="-p no:anyio"
