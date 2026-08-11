#!/usr/bin/env python3

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_uav_coppelia_api'
    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The UAV name used for namespacing.',
    ))

    # #} end of uav_name

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start as a standalone container or load into an existing container.'
    ))

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    ld.add_action(DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    ))

    # #} end of container_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'true'),
        description='Should the node subscribe to sim time?',
    ))

    # #} end of use_sim_time

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # Configuration files (the first one has priority and overrides subsequent ones)
    configs = [
        this_pkg_path + '/config/hw_api_plugin.yaml',
        get_package_share_directory('mrs_uav_hw_api') + '/config/hw_api.yaml',
    ]

    # #{ default node

    default_node = ComposableNode(
        package='mrs_uav_hw_api',
        plugin='mrs_uav_hw_api::HwApiManager',
        namespace=uav_name,
        name='hw_api',
        parameters=[
            {'uav_name': uav_name},
            {'topic_prefix': ['/', uav_name]},
            {'use_sim_time': use_sim_time},
            {'configs': configs},
            {'custom_config': custom_config},
        ],
        remappings=[
            ('~/errors', 'errors'),
        ],
        extra_arguments=[
            {'use_intra_process_comms': True}
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[default_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of default node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name='hw_api_container',
        package='rclcpp_components',
        executable='component_container_events_cbg',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[default_node],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of standalone container

    return ld
