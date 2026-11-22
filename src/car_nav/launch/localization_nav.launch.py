#!/usr/bin/env python3
"""官方标准链路 · 阶段 B: 静态地图定位 + Nav2 导航 (AMCL 提供 map->odom)。

官方架构关键点:
  1. SLAM 已停止, map_server + AMCL 接管 map->odom (两个定位源绝不同时运行);
  2. Gazebo 不在本文件中启动! 阶段 A 的 Gazebo 保持运行, 机器人位姿在
     世界中连续, 本文件只叠加定位与导航栈;
  3. 初始位姿由 run_official_pipeline.sh 依据 "slam map->odom TF" 与
     "/odom 当前里程计位姿" 计算后, 通过 initial_x/initial_y/initial_yaw
     传入 (机器人探索结束时通常已不在出生点)。

用法:
  ros2 launch car_nav localization_nav.launch.py \
      map:=/home/yzh/car_maps/official_map.yaml \
      initial_x:=-4.5 initial_y:=-4.5 initial_yaw:=0.0 run_mission:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 与 nav_demo.launch.py obstacle 场景一致的 5 个巡逻航点
# (tests/pick_waypoints.py 在 GT 图上挑选, 离障碍 >= 0.8m)
WAYPOINTS_X = [-0.51, 4.59, -5.01, 4.89, 3.99]
WAYPOINTS_Y = [-1.55, 5.05, 4.75, -4.55, 0.25]
SPAWN = (-4.5, -4.5)


def launch_setup(context):
    car_nav_share = get_package_share_directory('car_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_mission = LaunchConfiguration('run_mission')
    initial_x = LaunchConfiguration('initial_x').perform(context)
    initial_y = LaunchConfiguration('initial_y').perform(context)
    initial_yaw = LaunchConfiguration('initial_yaw').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    if not os.path.exists(map_file):
        raise RuntimeError(f'地图文件不存在: {map_file}')

    params_file = os.path.join(car_nav_share, 'config', 'nav2_burger.yaml')

    # 官方 bringup 两个 launch: localization(map_server+AMCL) + navigation
    nav2_stack = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),
    ])

    # AMCL 激活后立即播种初始位姿 (探索结束位置, 由编排脚本计算传入)
    initial_pose = Node(
        package='car_nav',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{'x': float(initial_x), 'y': float(initial_y),
                     'yaw': float(initial_yaw), 'use_sim_time': True}],
        output='screen',
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(car_nav_share, 'rviz', 'nav_demo.rviz')],
        parameters=[{'use_sim_time':
                     ParameterValue(use_sim_time, value_type=bool)}],
        output='screen',
    )

    # 航点巡逻任务 (可选): Nav2 完全激活后启动
    yaws = [0.0] * len(WAYPOINTS_X)
    mission = Node(
        package='car_nav',
        executable='mission_runner',
        name='mission_runner',
        parameters=[{
            'use_sim_time': True,
            'waypoint_x': [float(v) for v in WAYPOINTS_X],
            'waypoint_y': [float(v) for v in WAYPOINTS_Y],
            'waypoint_yaw': yaws,
            'initial_x': float(initial_x),
            'initial_y': float(initial_y),
            'initial_yaw': float(initial_yaw),
        }],
        output='screen',
        condition=IfCondition(run_mission),
    )

    return [
        nav2_stack,
        TimerAction(period=15.0, actions=[initial_pose]),
        TimerAction(period=20.0, actions=[rviz_cmd]),
        TimerAction(period=30.0, actions=[mission]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=
                              '/home/yzh/car_maps/official_map.yaml'),
        DeclareLaunchArgument('initial_x', default_value=str(SPAWN[0])),
        DeclareLaunchArgument('initial_y', default_value=str(SPAWN[1])),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('run_mission', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
