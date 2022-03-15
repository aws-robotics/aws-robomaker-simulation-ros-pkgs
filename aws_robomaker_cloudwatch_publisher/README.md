# AWS RoboMaker CloudWatch Publisher Package

This package contains utility classes to publish metrics to AWS CloudWatch, as well as ROS nodes and launch files to include in customer launch files.

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

## Install

1. Git clone the repository containing this repo into your workspace
```bash
cd <workspace_dir>/src
git clone https://github.com/aws-robotics/aws-robomaker-simulation-ros-pkgs.git
```

2. Install the required Python dependencies:
```bash
cd <workspace_dir>/src/aws-robomaker-simulation-ros-pkgs/aws_robomaker_cloudwatch_publisher
pip install -r requirements.txt
```

3. Include the required launch file from the package into yours:
```xml
<include file="$(find aws_robomaker_cloudwatch_publisher)/launch/rtf_cloudwatch_publisher.launch">
    <arg name="rate" value="1" />
    <arg name="gazebo_perf_metrics_topic" default="/gazebo/aws/perf_metrics" />
    <arg name="gazebo_world_stats_topic" default="/gazebo/default/world_stats" />
</include>
```

4. Build your workspace as you normally would
```bash
cd <workspace_dir>
catkin_make
source devel/setup.bash
```

## Usage

Set the environment variable `AWS_REGION` to the correct region, then launch your simulation application launch file as you normally would.
Note that without this environment variable set, the node will crash.

The CloudWatch publisher makes use of the AWS RoboMaker Gazebo Bridge, which is a small package that connects to the Gazebo C++ API and publishes certain information into ROS topics. The launch file for this package launches the Gazebo bridge using the arguments supplied.

### Arguments

Given the launch file block from above:

```xml
<include file="$(find aws_robomaker_cloudwatch_publisher)/launch/rtf_cloudwatch_publisher.launch">
    <arg name="rate" value="1" />
    <arg name="gazebo_perf_metrics_topic" default="/gazebo/aws/perf_metrics" />
    <arg name="gazebo_world_stats_topic" default="/gazebo/default/world_stats" />
</include>
```

The arguments are as follows:

1. `rate`: the frequency in Hz at which to publish to CloudWatch, given in Hz (default 1 Hz). If this is faster than the rate available from Gazebo, warnings will be produced about the frequency being set too high.
2. `gazebo_perf_metrics_topic`: the topic that Gazebo Bridge should publish on, and CloudWatch Publisher should subscribe on (default `/gazebo/aws/perf_metrics`).
3. `gazebo_world_stats_topic`: the topic that Gazebo Bridge should use to subscribe to Gazebo information via its API (as opposed to via ROS, where this topic will not be available). Default value is `/gazebo/default/world_stats`.

All arguments are optional as the default values should be sufficient for normal usage. Arguments may be changed to the user's preference, but changing the Gazebo Bridge topic is likely to cause no information to be published; modify this argument with care.

## License

This library is licensed under the Apache 2.0 License.
