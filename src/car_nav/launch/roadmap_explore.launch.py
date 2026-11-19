#!/usr/bin/env python3
"""基于 roadmap_explorer 的自主探索建图（替换原 frontier_explore.py）。

工作流:
  Gazebo + TurtleBot3  →  slam_toolbox 在线建图
                       →  Nav2 navigation (无 AMCL, 由 slam_toolbox 提供 map->odom)
                       →  roadmap_explorer 选 frontier + 发 NavigateToPose 给 Nav2
                       →  Nav2 DWB 局部避障执行

用法:
  ros2 launch car_nav roadmap_explore.launch.py
  # 建图完成后另开终端保存:
  #   ros2 run nav2_map_server map_saver_cli -f ~/maps/obstacle_field_slam

需要环境变量:
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 必须! FastDDS 会 segfault
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    car_nav_share = get_package_share_directory('car_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    roadmap_explorer_share = get_package_share_directory('roadmap_explorer')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 包含现有 slam_demo (Gazebo + spawn + slam_toolbox + RViz)
    slam_demo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_nav_share, 'launch', 'slam_demo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
    )

    # Nav2 navigation (不带 amcl/map_server, 由 slam_toolbox 提供定位与地图)
    nav2_params = os.path.join(car_nav_share, 'config', 'nav2_burger.yaml')
    nav2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'True',
        }.items(),
    )

    # roadmap_explorer 节点
    explorer_params = os.path.join(car_nav_share, 'config', 'roadmap_obstacle_field.yaml')
    explorer_node = Node(
        package='roadmap_explorer',
        executable='roadmap_exploration_server',
        name='roadmap_explorer_node',
        output='screen',
        parameters=[explorer_params, {'use_sim_time': use_sim_time}],
    )

    # lifecycle manager 激活 roadmap_explorer_node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_exploration',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['roadmap_explorer_node']},
            {'use_sim_time': use_sim_time},
        ],
    )

    return [
        slam_demo_cmd,
        nav2_navigation_cmd,
        explorer_node,
        lifecycle_manager,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='obstacle'),
        DeclareLaunchArgument('x_pose', default_value='-4.5'),
        DeclareLaunchArgument('y_pose', default_value='-4.5'),
        OpaqueFunction(function=launch_setup),
    ])
