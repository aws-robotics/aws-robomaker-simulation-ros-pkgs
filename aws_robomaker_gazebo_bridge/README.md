# AWS RoboMaker Gazebo Launch Package

This package contains ROS nodes that can connect to a Gazebo instance on the same instance and publish data to known ROS topics.

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

## Install

1. Git clone the repository containing this repo into your workspace
```bash
cd <workspace_dir>/src
git clone https://github.com/aws-robotics/aws-robomaker-simulation-ros-pkgs.git
```

2. Include this package's launch file into yours (typically this would be where you used a launch file from `gazebo_ros`):
```xml
<include file="$(find aws_robomaker_gazebo_bridge)/launch/perf_metrics_bridge.launch">
    <param name="gazebo_perf_metrics_topic" value="$(arg gazebo_perf_metrics_topic)" />
    <param name="gazebo_world_stats_topic" value="$(arg gazebo_world_stats_topic)" />
</include>
```

3. Build your workspace as you normally would
```bash
cd <workspace_dir>
catkin_make
source devel/setup.bash
```

## Usage

Now you can launch this node alongside a Gazebo instance to start seeing metrics being produced,

## License

This library is licensed under the Apache 2.0 License. 
