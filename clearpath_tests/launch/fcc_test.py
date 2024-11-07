#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath setup path',
    )
]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_tests = get_package_share_directory('clearpath_tests')

    setup_path = LaunchConfiguration('setup_path')

    robot_yaml = PathJoinSubstitution(
        [setup_path, 'robot.yaml']
    )


    # Read robot YAML
    config = read_yaml(robot_yaml.perform(context))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    tests = GroupAction(
        [
            PushRosNamespace(namespace),
            # Drive test
            Node(
                package='clearpath_tests',
                executable='drive_test',
                output='screen',
                arguments=['-s', setup_path]
            ),
          # Light test
            Node(
                package='clearpath_tests',
                executable='light_test',
                output='screen',
                arguments=['-s', setup_path]
            ),
        ]
    )

    return [tests]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
