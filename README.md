# Distributed Robot Perception Workspace

This is the ROS 2 workspace for the **Distributed Robot Perception** project. The workspace contains several packages that are essential for simulating and controlling the TurtleBot3 or Shelfino robot in various environments.

## Packages

The following packages are included in this workspace:

- `name_pkg`: Contains ...

## How to Build the Workspace

To build the workspace, follow the following steps:

1- clone repo inside yout workspace of ros2
2- in your workspace directory do the `colcon build`
```bash
colcon build --symlink-install && source setup/install.bash

```
3- run the launch file