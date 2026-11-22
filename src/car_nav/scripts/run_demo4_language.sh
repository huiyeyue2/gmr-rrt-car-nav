#!/bin/bash
# ============ 演示 4: 自然语言任务（打字下指令, 机器人导航+相机观察） ============
# 用法: bash run_demo4_language.sh
# 需要: 已保存的障碍场地图; burger_cam 模型(自带相机); 可选 AI 后端配置:
#       ~/.ai_backend.json 填入 GLM api_key 后启用 LLM 解析与 VLM 看图回答,
#       未配置则自动降级为规则解析 + 抓拍存图(离线模式)
set -e
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../../../install/setup.bash"
export TURTLEBOT3_MODEL=burger_cam          # 带相机的 TurtleBot3

MAP_DIR="$(dirname "$0")/../maps"
CAR_MAPS="$HOME/car_maps"
mkdir -p "$CAR_MAPS"
cp -u "$MAP_DIR"/*.pgm "$MAP_DIR"/*.yaml "$CAR_MAPS"/
echo ">>> 生效地图: $(head -c 32 "$CAR_MAPS"/obstacle_field.pgm | tr '\n' ' ')"

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

echo ">>> 启动导航底座 (Gazebo + Nav2 + RViz, 含相机画面面板)"
ros2 launch car_nav nav_demo.launch.py &
sleep 25

echo ">>> 启动自然语言任务接口 (交互模式, /help 查看示例)"
ros2 run car_nav language_mission
