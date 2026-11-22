#!/bin/bash
# ============ 演示 2: 航点导航巡逻任务（障碍物场地） ============
# 2026-11-22: 增加 git 学习演练注释行（无功能影响）
# 用法: bash run_demo2_mission.sh
# 需要: 演示 1 保存的地图 (car_ws/src/car_nav/maps/obstacle_field.*)
set -e
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../../../install/setup.bash"
export TURTLEBOT3_MODEL=burger

# 地图同步到 ASCII 路径(Humble launch 参数不兼容非 ASCII 路径)
# 用 -u 只覆盖更新的文件, 避免包里的旧/残缺地图把 car_maps 里已建好的好图冲掉
MAP_DIR="$(dirname "$0")/../maps"
CAR_MAPS="$HOME/car_maps"
mkdir -p "$CAR_MAPS"
cp -u "$MAP_DIR"/*.pgm "$MAP_DIR"/*.yaml "$CAR_MAPS"/
echo ">>> 生效地图: $(basename "$MAP_DIR"/obstacle_field.pgm) $(head -c 32 "$CAR_MAPS"/obstacle_field.pgm | tr '\n' ' ')"

echo ">>> 清理残留进程 (demo1 结束后 Gazebo/RViz 保持运行供查看, 必须先清)"
pkill -f '[r]oadmap' || true
pkill -f '[s]lam_toolbox' || true
pkill -f '[n]av2' || true
pkill -f '[r]viz2' || true
pkill -f '[g]zserver' || true
pkill -f '[g]zclient' || true
sleep 3
pkill -9 -f '[g]zserver' || true
pkill -9 -f '[g]zclient' || true

echo ">>> 启动导航演示 (Gazebo + Nav2 + 自动初始位姿 + 航点任务)"
echo "    5 个巡逻航点, 终端实时打印 剩余距离/到达 状态"
ros2 launch car_nav nav_demo.launch.py run_mission:=true
