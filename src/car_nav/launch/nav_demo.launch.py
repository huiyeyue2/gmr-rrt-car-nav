#!/usr/bin/env python3
"""导航任务演示（图3 效果）：Gazebo + Nav2 + AMCL + 航点巡逻 + GMR-RRT*。

场景选择(scene:=): obstacle(默认, 障碍物场地) / house(房屋, 需先建好 house 地图)

用法:
  ros2 launch car_nav nav_demo.launch.py                       # 手动在 RViz 点目标
  ros2 launch car_nav nav_demo.launch.py run_mission:=true     # 自动航点巡逻
  ros2 launch car_nav nav_demo.launch.py scene:=house run_mission:=true
  # GMR-RRT* 演示(另开终端), 在 RViz 用 "2D Goal Pose" 点目标:
  ros2 run car_nav gmr_planner_node --ros-args -p use_sim_time:=true
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

# 各场景: (gzserver 世界文件, 出生点, 地图, 巡逻航点 world 坐标)
SCENES = {
    'obstacle': {
        'world': 'obstacle_field.world',          # car_nav/worlds/ 下
        'spawn': (-4.5, -4.5),
        'map': 'obstacle_field.yaml',
        # 航点由 tests/pick_waypoints.py 自动挑选, 并加严到离障碍 >=0.8m:
        # Nav2 全局代价地图的膨胀半径(0.35)+内接半径让 0.4m 净空的点在栅格
        # 上变成致命格, NavFn 会在机器人已到达时仍报 failed to create plan
        'waypoints_x': [-0.51, 4.59, -5.01, 4.89, 3.99],
        'waypoints_y': [-1.55, 5.05, 4.75, -4.55, 0.25],
    },
    'house': {
        'world': None,                            # 用 turtlebot3_house.launch.py
        'spawn': (-2.0, -0.5),
        'map': 'house.yaml',
        # 由 _vmfix/pick_house_wps.py 在 house.pgm 上挑: 离墙/家具 >=0.5m 净空、
        # 四点分散在不同房间, 保证 Nav2 能穿过门洞走一圈
        'waypoints_x': [-5.99, -0.89, 2.61, 6.06],
        'waypoints_y': [3.30, -2.85, -3.16, 1.20],
    },
}


def launch_setup(context):
    car_nav_share = get_package_share_directory('car_nav')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_mission = LaunchConfiguration('run_mission')
    scene_name = LaunchConfiguration('scene').perform(context)
    scene = SCENES.get(scene_name)
    if scene is None:
        raise RuntimeError(f'未知场景 {scene_name}, 可选: {list(SCENES)}')

    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    # 自定义参数: 在官方 burger.yaml 基础上把局部代价地图扩到 12x12m
    # (RRT* 路径绕障弯度大, 3x3m 窗口会丢路径导致 DWB 中止)
    default_params = os.path.join(car_nav_share, 'config', 'nav2_burger.yaml')
    if not os.path.exists(default_params):
        default_params = os.path.join(tb3_nav2, 'param', 'humble',
                                      model + '.yaml')
    # 注意: ROS 2 Humble 的 launch 参数序列化会把非 ASCII 路径转成 \uXXXX 字面量
    # (如 F盘 -> Fu76D8), 因此地图 yaml 优先用 ASCII 镜像目录 /home/yzh/car_maps/
    map_file = os.path.join('/home/yzh/car_maps', scene['map'])
    if not os.path.exists(map_file):
        map_file = os.path.join(car_nav_share, 'maps', scene['map'])

    # ---- Gazebo: 障碍场用自有 world, 房屋用自带 launch ----
    if scene['world'] is not None:
        gazebo_ros = get_package_share_directory('gazebo_ros')
        world_file = os.path.join(car_nav_share, 'worlds', scene['world'])
        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items())
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
            launch_arguments={'x_pose': str(scene['spawn'][0]),
                              'y_pose': str(scene['spawn'][1])}.items())
        gazebo_part = [gzserver, gzclient, rsp, spawn_tb3]
    else:
        gazebo_part = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo, 'launch',
                             'turtlebot3_house.launch.py')))]

    # 之前 include turtlebot3_navigation2/launch/navigation2.launch.py, 它会
    # 无条件再启动一个 tb3_navigation2.rviz(该配置引用了本机 Nav2 没有的
    # Docking/Selector 面板, 一开就刷 plugin 报错), 桌面上会出现两个 RViz。
    # 这里直接用 nav2_bringup 的 localization + navigation 两个 launch, 它们
    # 不含 rviz, 参数与原来那份 TB3 launch 完全等价。
    nav2_bringup = get_package_share_directory('nav2_bringup')
    navigation2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': default_params,
                'autostart': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': default_params,
                'autostart': 'true',
            }.items(),
        ),
    ])

    # Nav2 起来后自动设置初始位姿(场景出生点)
    initial_pose = Node(
        package='car_nav',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{'x': float(scene['spawn'][0]),
                     'y': float(scene['spawn'][1]),
                     'yaw': 0.0, 'use_sim_time': True}],
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

    # 航点巡逻任务(可选): 等 Nav2 完全激活后启动
    yaws = [0.0] * len(scene['waypoints_x'])
    mission = Node(
        package='car_nav',
        executable='mission_runner',
        name='mission_runner',
        parameters=[{
            'use_sim_time': True,
            'waypoint_x': [float(v) for v in scene['waypoints_x']],
            'waypoint_y': [float(v) for v in scene['waypoints_y']],
            'waypoint_yaw': yaws,
            # 出生点必须和 Gazebo 里的一致, 否则 AMCL 会被拉到原点
            'initial_x': float(scene['spawn'][0]),
            'initial_y': float(scene['spawn'][1]),
            'initial_yaw': 0.0,
        }],
        output='screen',
        condition=IfCondition(run_mission),
    )

    return gazebo_part + [
        TimerAction(period=8.0, actions=[navigation2]),
        TimerAction(period=22.0, actions=[initial_pose, rviz_cmd]),
        TimerAction(period=35.0, actions=[mission]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('run_mission', default_value='false'),
        DeclareLaunchArgument('scene', default_value='obstacle'),
        OpaqueFunction(function=launch_setup),
    ])
