#!/usr/bin/env python3
"""一键启动: 障碍场 Gazebo 世界 + Nav2(定位+导航) + RViz + 自写 A* 规划节点。

和 nav_demo.launch.py 共用同一套世界/地图/参数/RViz 配置, 区别是这里额外起
astar_planner: 在 RViz 里用 "A* Goal" 工具点目标, 就能看到自写 A* 的全局
路径(发布在 /car_nav/plan), 用来和 GMR-RRT* / NavFn 做对比。

注意: 这里不再 include turtlebot3_navigation2/launch/navigation2.launch.py,
它会无条件多开一个 tb3_navigation2.rviz(引用本机 Nav2 没有的 Docking/Selector
面板, 一开就刷 plugin 报错), 导致桌面上出现两个 RViz。

用法:
  ros2 launch car_nav car_nav_demo.launch.py
  ros2 launch car_nav car_nav_demo.launch.py init_x:=-4.5 init_y:=-4.5 init_yaw:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')
    map_arg = LaunchConfiguration('map')

    car_nav_share = get_package_share_directory('car_nav')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    default_params = os.path.join(car_nav_share, 'config', 'nav2_burger.yaml')
    if not os.path.exists(default_params):
        default_params = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param', 'humble', model + '.yaml')

    # ROS 2 Humble 的 launch 参数序列化会把非 ASCII 路径转成 \uXXXX 字面量
    # (如含中文的目录 -> \uXXXX), 因此地图 yaml 优先用 ASCII 镜像目录 ~/car_maps/
    default_map = os.path.expanduser('~/car_maps/obstacle_field.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.join(car_nav_share, 'maps', 'obstacle_field.yaml')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5'}.items(),
    )

    gazebo_ros = get_package_share_directory('gazebo_ros')
    custom_world = os.path.join(car_nav_share, 'worlds', 'obstacle_field.world')
    if os.path.exists(custom_world):
        world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': custom_world}.items())
        gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')))
        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch',
                             'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items())
        spawn_tb3 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch',
                             'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': '-4.5', 'y_pose': '-4.5'}.items())
        world = [world, gzclient, rsp, spawn_tb3]

    navigation2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch',
                             'localization_launch.py')),
            launch_arguments={
                'map': map_arg,
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch',
                             'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
            }.items(),
        ),
    ])

    # Nav2 起来后自动设置初始位姿, 替代手动点 2D Pose Estimate
    initial_pose = Node(
        package='car_nav',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{
            'x': ParameterValue(init_x, value_type=float),
            'y': ParameterValue(init_y, value_type=float),
            'yaw': ParameterValue(init_yaw, value_type=float),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
    )

    planner = Node(
        package='car_nav',
        executable='astar_planner',
        name='astar_planner',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(car_nav_share, 'rviz', 'nav_demo.rviz')],
        parameters=[{'use_sim_time':
                     ParameterValue(use_sim_time, value_type=bool)}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('init_x', default_value='-4.5'),
        DeclareLaunchArgument('init_y', default_value='-4.5'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),

        world,
        TimerAction(period=8.0, actions=[navigation2]),
        TimerAction(period=22.0, actions=[initial_pose, planner, rviz]),
    ])