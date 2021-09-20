# Point Cloud Global to Local

## Installation

Create catkin workspace directory at `~/catkin_ws` and clone repository into `~/catkin_ws/src`.

### Install Dependencies

```bash
rosdep install -y --from-paths src --rosdistro noetic
```

### Build Package

```bash
catkin_make
```

## Usage

Run node:

```bash
rosrun point_cloud_global_to_local node
```

## Published Topics

The ROS node publishes several topics:

## Helpful commands

Source workspace

```bash
source ~/catkin_ws/devel/setup.bash
```
