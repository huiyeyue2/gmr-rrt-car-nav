#!/usr/bin/env python3
"""官方标准链路 · 阶段 A: SLAM 建图 + 自主探索。

官方架构 (slam_toolbox + Nav2):
  建图阶段由 slam_toolbox 发布 map->odom TF, map_server/AMCL 不启动
  (两个定位源绝不同时运行)。
  地图保存由 run_official_pipeline.sh 在停 SLAM 之前用 map_saver_cli 完成。

本文件复用演示 1 的标准探索链路 (roadmap_explore.launch.py):
  Gazebo(GUI) + TurtleBot3 + slam_toolbox 在线建图
  + Nav2 navigation (无 AMCL) + roadmap_explorer 自主探索 + RViz

用法:
  ros2 launch car_nav slam_nav.launch.py
需要环境变量:
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 必须, FastDDS 会 segfault
  export TURTLEBOT3_MODEL=burger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    car_nav_share = get_package_share_directory('car_nav')

    # 阶段 A 主体 = 演示 1 标准探索链路 (Gazebo + slam_toolbox + Nav2 + explorer)
    slam_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_nav_share, 'launch', 'roadmap_explore.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
    )
    return [slam_nav_cmd]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='obstacle'),
        DeclareLaunchArgument('x_pose', default_value='-4.5'),
        DeclareLaunchArgument('y_pose', default_value='-4.5'),
        OpaqueFunction(function=launch_setup),
    ])
