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
    <arg name="publish_rate_frequency" value="$(arg rate)"/>
    <arg name="gazebo_topic" value="$(arg gazebo_topic)"/>
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

## License

This library is licensed under the Apache 2.0 License.
