FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=:1

# Dev tools
RUN apt-get update && apt-get install -y \
    build-essential cmake git curl \
    python3-colcon-common-extensions \
    python3-rosdep python3-pip

# Gazebo + TurtleBot3
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3* \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces

# noVNC + display stack
RUN apt-get update && apt-get install -y \
    xvfb x11vnc novnc xdotool

# Set noVNC default scaling
RUN sed -i "s/resize', 'off'/resize', 'scale'/" \
    /usr/share/novnc/app/ui.js

# Workspace
RUN mkdir -p /ros2_ws/packages
WORKDIR /ros2_ws

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
COPY start.sh /start.sh
RUN chmod +x /entrypoint.sh /start.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start.sh"]