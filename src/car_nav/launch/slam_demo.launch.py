#!/usr/bin/env python3
"""SLAM 实时建图演示（图1/图2 效果）：TurtleBot3 + slam_toolbox + RViz。

用法:
  ros2 launch car_nav slam_demo.launch.py                          # 默认障碍场
  ros2 launch car_nav slam_demo.launch.py world:=house \\
      x_pose:=-2.0 y_pose:=-0.5                                    # 房屋场景建图
  # world 也可以传任意 .world 文件的绝对路径

  # 建图完成后另开终端保存地图:
  #   ros2 run nav2_map_server map_saver_cli -f ~/maps/obstacle_field
  # 手动遥控建图:
  #   ros2 run turtlebot3_teleop teleop_keyboard
  # 自主探索建图:
  #   ros2 run car_nav auto_explore --ros-args -p use_sim_time:=true -p duration:=300.0
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

# 内置世界别名: obstacle = 本项目手写障碍场, house = turtlebot3_gazebo 自带房屋


def launch_setup(context):
    car_nav_share = get_package_share_directory('car_nav')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    world_value = LaunchConfiguration('world').perform(context)
    if world_value == 'obstacle':
        world_file = os.path.join(car_nav_share, 'worlds', 'obstacle_field.world')
    elif world_value == 'house':
        # 房屋世界直接用 turtlebot3_gazebo 自带的（其模型由该包的环境钩子解析）
        world_file = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'worlds', 'turtlebot3_house.world')
    elif os.path.isabs(world_value):
        world_file = world_value
    else:
        world_file = os.path.join(car_nav_share, 'worlds', world_value)

    slam_params = os.path.join(car_nav_share, 'config', 'slam_online_async.yaml')
    rviz_config = os.path.join(car_nav_share, 'rviz', 'slam_demo.rviz')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params}.items(),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        output='screen',
    )

    return [
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        slam_cmd,
        rviz_cmd,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='obstacle'),
        DeclareLaunchArgument('x_pose', default_value='-4.5'),
        DeclareLaunchArgument('y_pose', default_value='-4.5'),
        OpaqueFunction(function=launch_setup),
    ])
