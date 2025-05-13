FROM osrf/ros:humble-desktop-full

# Create a new user 'rosuser' with a home directory
# Install essential ROS 2 and Python tools
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
    ros-humble-ros2-controllers && \
    rm -rf /var/lib/apt/lists/*

# Set the workspace directory
WORKDIR /ros2_ws

# # Create the 'src' directory inside the workspace
# RUN mkdir -p ros2_ws/src_DRP
# COPY src_DRP ros2_ws/src_DRP
# Source the ROS 2 setup script in the user's bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# Default command to run when the container starts
CMD ["bash"]