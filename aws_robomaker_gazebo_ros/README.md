# AWS RoboMaker Gazebo Launch Package

This package contains ROS launch files that integrate with AWS RoboMaker's runtime features for Gazebo. The launch files are intended as drop in replacements for the default launch files provided by `gazebo_ros`

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

## Install

1. Git clone the repository containing this repo into your workspace
```bash
cd <workspace_dir>/src
git clone https://github.com/aws-robotics/aws-robomaker-simulation-ros-pkgs.git
```

2. Include this package's launch file into yours (typically this would be where you used a launch file from `gazebo_ros`):
```xml
<include file="$(find aws_robomaker_gazebo_ros)/launch/launch_world.launch">
    <arg name="gui" value="$(arg gui)"/>
    <arg name="world_name" value="/path/to/world/sdf/file"/>
</include>
```

3. Build your workspace as you normally would
```bash
cd <workspace_dir>
rosdep install --from-paths . --ignore-src -r -y
colcon build
```

## Usage

Now you can launch your simulation application launch file as you normally would.

You can also `colcon bundle` as you normally would to bring this simulation application onto AWS RoboMaker.


## License

This library is licensed under the Apache 2.0 License. 
