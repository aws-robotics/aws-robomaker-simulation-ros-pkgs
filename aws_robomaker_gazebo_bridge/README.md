# AWS RoboMaker Gazebo Bridge Package

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
<include file="$(find aws_robomaker_cloudwatch_publisher)/launch/rtf_cloudwatch_publisher.launch">
    <arg name="rate" value="1" />
    <arg name="gazebo_perf_metrics_topic" value="/gazebo/aws/perf_metrics" />
    <arg name="gazebo_world_stats_topic" value="/gazebo/default/world_stats" />
</include>
```

3. Build your workspace as you normally would
```bash
cd <workspace_dir>
catkin_make
source devel/setup.bash
```

## Usage

This node may now be launched alongside a Gazebo instance to start seeing metrics being produced on the `/gazebo/aws/perf_metrics` ROS topic (by default).

### Arguments

Given the launch file block from above:

```xml
<include file="$(find aws_robomaker_cloudwatch_publisher)/launch/rtf_cloudwatch_publisher.launch">
    <arg name="rate" value="1" />
    <arg name="gazebo_perf_metrics_topic" value="/gazebo/aws/perf_metrics" />
    <arg name="gazebo_world_stats_topic" value="/gazebo/default/world_stats" />
</include>
```

The arguments are as follows:

1. `gazebo_perf_metrics_topic`: the topic that Gazebo Bridge should publish on (default `/gazebo/aws/perf_metrics`).
2. `gazebo_world_stats_topic`: the topic that Gazebo Bridge should use to subscribe to Gazebo information via its API (as opposed to via ROS, where this topic will not be available). Default value is `/gazebo/default/world_stats`.

All arguments are optional as the default values should be sufficient for normal usage. Arguments may be changed to the user's preference, but changing the Gazebo Bridge topic is likely to cause no information to be published; modify this argument with care.

## License

This library is licensed under the Apache 2.0 License. 
