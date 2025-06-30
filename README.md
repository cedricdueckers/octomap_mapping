octomap_mapping ![CI](https://github.com/OctoMap/octomap_mapping/workflows/CI/badge.svg)
===============

ROS stack for mapping with OctoMap, contains the `octomap_server` package.

The main branch for ROS1 Kinetic, Melodic, and Noetic is `kinetic-devel`.

The main branch for ROS2 Foxy and newer is `ros2`.

### Usage

#### Run RGB OctoMap Server

```
ros2 launch octomap_server rgb_octomap_server.launch.py
```

This runs the colored OctoMap server with a provided colored point cloud (refer to the launch file for configuration). 

It publishes the colored OctoMap on the topic `/color_octomap`.

#### Save RGB OctoMap
```
ros2 run octomap_server rgb_octomap_saver_node --ros-args -p octomap_path:="path.ot"
```

#### Save octomap

```
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=(path for saving octomap)
```
Note: The extension of octomap path should be `.bt` or `.ot`





