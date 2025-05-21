FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-twist-mux && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    nano \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-* \
    ros-humble-ros-gz* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-twist-mux \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-controllers && \
    rm -rf /var/lib/apt/lists/*

# Set the workspace directory
WORKDIR /ros2_ws

# # Create the 'src' directory inside the workspace
# RUN mkdir -p ros2_ws/src_DRP
# COPY src_DRP ros2_ws/src_DRP
# Source the ROS 2 setup script in the user's bashrc

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc


# Default command to run when the container starts
CMD ["bash"]