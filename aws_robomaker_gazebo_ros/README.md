# AWS RoboMaker Gazebo Launch Package

This package contains ROS-2 launch files that integrate with AWS RoboMaker's runtime features for Gazebo. The launch files are intended as drop in replacements for the default launch files provided by `gazebo_ros`

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

## Install

You will need the source code in your workspace. 

1. Git clone the repository containing this repo into your workspace
```bash
cd <simulation_ws>/src
git clone --single-branch --branch gazebolaunch-ros2 https://github.com/aws-robotics/aws-robomaker-simulation-ros-pkgs.git
```

2. Include this package's launch file into yours (typically this would be where you used a launch file from `gazebo_ros`):
```
launch.LaunchDescription(
    launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'aws_robomaker_gazebo_ros'),
                'launch',
                'launch_world.py',
            )
        ),
        launch_arguments={
            'gui': launch.substitutions.LaunchConfiguration('gui'),
            'world_name': os.path.join(
                get_package_share_directory('aws_robomaker_worldforge_worlds'),
                'worlds',
                'generation_sm885pycnlpf_world_1',
                'generation_sm885pycnlpf_world_1.world')
        }.items()
    )
)
```

3. Build your workspace as you normally would
```bash
# build for ROS
cd <simulation_ws>
rosdep install --from-paths . --ignore-src -r -y
colcon build
```

## Usage

Now you can launch your simulation application as you normally would.

You can also `colcon bundle` as you normally would to bring this simulation application onto AWS RoboMaker.


## License

This library is licensed under the Apache 2.0 License. 
