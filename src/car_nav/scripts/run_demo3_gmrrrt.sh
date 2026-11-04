#!/bin/bash
# ============ 演示 3: GMR-RRT* 算法驱动导航（论文核心） ============
# 用法: bash run_demo3_gmrrrt.sh
# 需要: 演示 1 保存的地图。启动后在 RViz 用 "2D Goal Pose" 点一个目标,
#       即触发: demo轨迹 -> GMM -> GMR -> GMR-RRT* -> 机器人沿算法路径行驶
set -e
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../../../install/setup.bash"
export TURTLEBOT3_MODEL=burger

MAP_DIR="$(dirname "$0")/../maps"
mkdir -p /home/yzh/car_maps
cp -u "$MAP_DIR"/*.pgm "$MAP_DIR"/*.yaml /home/yzh/car_maps/
echo ">>> 生效地图: $(head -c 32 /home/yzh/car_maps/obstacle_field.pgm | tr '\n' ' ')"

echo ">>> 启动导航底座 (Gazebo + Nav2 + RViz)"
ros2 launch car_nav nav_demo.launch.py &
sleep 25

echo ">>> 启动 GMR-RRT* 规划节点 (论文链路, 含 NavFn 对比模式)"
echo "    在 RViz 工具栏选 'GMR Goal'(2D Goal Pose 图标, 发布 /gmr/goal_pose),"
echo "    在地图空地点击并拖出方向; 标准 'Nav2 Goal' 触发 NavFn 对比"
ros2 run car_nav gmr_planner_node --ros-args -p use_sim_time:=true -p compare_navfn:=true
