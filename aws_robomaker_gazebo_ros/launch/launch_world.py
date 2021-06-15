import os
from os import environ
from os import pathsep
import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import Shutdown, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from ros2pkg.api import get_package_names


def generate_launch_description():

    world_sdf_path_override = 'null'
    if os.getenv('AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE') is not None:
        # check if worldforge world(s) exist in the customer workspace
        worldforge_world_pkg_count = os.getenv('AMENT_PREFIX_PATH','').count('aws_robomaker_worldforge_worlds')
        if (worldforge_world_pkg_count > 1):
            print("[Wrn] A AWS Robomaker worldforge world exists in your ROS environment. "
                "There is no guarantee that the worldforge world referenced via world ARN will be used "
                "during the execution of the simulation job. This message can be safely ignored if you "
                "are using a bundle to create the simulation job.")
        world_pkg_path = get_package_share_directory('aws_robomaker_worldforge_worlds')
        world_sdf_path_override = glob.glob(world_pkg_path + '/worlds' + '/**/generation_*_world_*.world', recursive = True)

    gazebo_ros_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world_name'),
            'gui': LaunchConfiguration('gui'),
            'extra_gazebo_args': LaunchConfiguration('extra_gazebo_args'),
            'server_required': LaunchConfiguration('server_required'),
            'verbose': LaunchConfiguration('verbose'),
            'pause': LaunchConfiguration('pause'),
            'physics': LaunchConfiguration('physics'),
            'record': LaunchConfiguration('record'),
            'record_encoding': LaunchConfiguration('record_encoding'),
            'record_path': LaunchConfiguration('record_path'),
            'record_period': LaunchConfiguration('record_period'),
            'record_filter': LaunchConfiguration('record_filter'),
            'lockstep': LaunchConfiguration('lockstep'),
            'help': LaunchConfiguration('help'),
            'play': LaunchConfiguration('play'),
            'seed': LaunchConfiguration('seed'),
            'iters': LaunchConfiguration('iters'),
            'minimal_comms': LaunchConfiguration('minimal_comms'),
            'profile': LaunchConfiguration('profile'),
            'gdb': LaunchConfiguration('gdb'),
            'valgrind': LaunchConfiguration('valgrind'),
            'init': LaunchConfiguration('init'),
            'factory': LaunchConfiguration('factory'),
            'force_system': LaunchConfiguration('force_system'),
        }.items()
    )


    ld_args = [
        DeclareLaunchArgument(
            'world_name',
            default_value=PythonExpression([LaunchConfiguration('world_name'),
                ' if "null" == ', world_sdf_path_override, ' else ', world_sdf_path_override]),
            description='Path to the world file with respect to GAZEBO_RESOURCE_PATH env variable.'
        ),
        DeclareLaunchArgument(
            'pause', default_value='false'
            description='Pause gazebo after loading the world if set to true'
        ),
        DeclareLaunchArgument(
            'extra_gazebo_args', default_value='',
            description='Extra arguments to be passed to Gazebo'
        ),
        DeclareLaunchArgument(
            'physics', default_value='ode',
            description='Specify a physics engine (ode|bullet|dart|simbody).'
        ),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Set "true" to record state data.'
        ),
        DeclareLaunchArgument(
            'record_encoding', default_value='',
            description='Specify compression encoding format for log data (zlib|bz2|txt).'
        ),
        DeclareLaunchArgument(
            'record_path', default_value='',
            description='Absolute path in which to store state data.'
        ),
        DeclareLaunchArgument(
            'record_period', default_value='',
            description='Specify recording period (seconds).'
        ),
        DeclareLaunchArgument(
            'record_filter', default_value='',
            description='Specify recording filter (supports wildcard and regular expression).'
        ),
        DeclareLaunchArgument(
            'server_required', default_value='false',
            description='Set "true" to shut down launch script when server is terminated'
        ),
        DeclareLaunchArgument(
            'lockstep', default_value='false',
            description='Set "true" to respect update rates'
        ),
        DeclareLaunchArgument(
            'help', default_value='false',
            description='Set "true" to produce gzserver help message.'
        ),
        DeclareLaunchArgument(
            'play', default_value='',
            description='Play the specified log file.'
        ),
         DeclareLaunchArgument(
            'seed', default_value='', description='Start with a given a random number seed.'
        ),
        DeclareLaunchArgument(
            'iters', default_value='', description='Specify number of iterations to simulate.'
        ),
        DeclareLaunchArgument(
            'minimal_comms', default_value='false',
            description='Set "true" to reduce TCP/IP traffic output.'
        ),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='Specify physics preset profile name from the options in the world file.'
        ),
        # Specific to gazebo_ros
        DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Set "true" to run gzserver with gdb'
        ),
        DeclareLaunchArgument(
            'valgrind', default_value='false',
            description='Set "true" to run gzserver with valgrind'
        ),
        DeclareLaunchArgument(
            'init', default_value='true',
            description='Set "false" not to load "libgazebo_ros_init.so"'
        ),
        DeclareLaunchArgument(
            'factory', default_value='true',
            description='Set "false" not to load "libgazebo_ros_factory.so"'
        ),
        DeclareLaunchArgument(
            'force_system', default_value='true',
            description='Set "false" not to load "libgazebo_ros_force_system.so"'
        ),
        gazebo_ros_ld
    ]

    return LaunchDescription(ld_args)
