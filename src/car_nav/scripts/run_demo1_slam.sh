#!/bin/bash
# ============ 演示 1: SLAM 实时建图（障碍物场地） ============
# 用法: 在虚拟机终端直接运行  bash run_demo1_slam.sh  [最多秒数]
#   $1 = 最长建图秒数（默认 0 = 一直跑到地图没有未知区域为止）
set -e
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../../../install/setup.bash"
export TURTLEBOT3_MODEL=burger

echo ">>> 启动 SLAM 建图 (RViz + Gazebo 窗口将出现在桌面)"
ros2 launch car_nav slam_demo.launch.py &
LAUNCH_PID=$!
sleep 15

# duration 是 double 类型参数: 传 20 会被 rclpy 拒绝
# (InvalidParameterTypeException: ... '20' of type 'INTEGER', expecting 'DOUBLE')
# 所以先把"整数秒数"补成 20.0
DURATION="${1:-0}"
case "$DURATION" in
  *.*) ;;
  *) DURATION="${DURATION}.0" ;;
esac
echo ">>> 启动前沿(frontier)自主探索: 会主动开向地图中所有未知区域"
echo "    (${DURATION} 秒后强制结束; 传 0 或不传 = 跑到未知区域探索完为止)"
echo "    也可 Ctrl+C 停止后改键盘遥控: ros2 run turtlebot3_teleop teleop_keyboard"
# 探索节点异常不应带走后面的存图: 失败只提示, 继续存图
if ! ros2 run car_nav frontier_explore \
       --ros-args -p use_sim_time:=true -p duration:="$DURATION"; then
  echo "!!! 探索节点退出异常, 继续保存当前地图" >&2
fi

# 存图: 单独存成 obstacle_field_slam.pgm, 不覆盖导航演示用的正式地图
# 原因: SLAM 建图会有累积漂移(本场地实测障碍物位置偏 0.2~0.4m、部分箱子缺失),
# 一旦导航演示直接用它, AMCL 会定位跑飞。正式地图 maps/obstacle_field.pgm
# 是按 worlds/obstacle_field.world 几何生成的, 与仿真世界严格一致。
MAP_BASE="$(dirname "$0")/../maps/obstacle_field_slam"
if [ -f "${MAP_BASE}.pgm" ]; then
  cp -f "${MAP_BASE}.pgm" "${MAP_BASE}.prev.pgm"
  cp -f "${MAP_BASE}.yaml" "${MAP_BASE}.prev.yaml"
  echo ">>> 已备份上一版建图结果到 $(basename "${MAP_BASE}").prev.pgm"
fi
echo ">>> 探索结束, 保存地图到 $MAP_BASE"
# map_saver_cli 只认 -f/-t/--occ/--free/--fmt/--mode,
# save_map_timeout 必须放在 --ros-args 之后; 默认 2s, 地图较大时会报
# "Failed to spin map subscription", 因此放宽到 15s。
SAVE_FAILED=0
if ros2 run nav2_map_server map_saver_cli -f "$MAP_BASE" \
     --ros-args -p save_map_timeout:=15.0; then
  # 打出实际存下的尺寸, 便于一眼看出地图是否空/残缺
  echo ">>> 地图已保存: ${MAP_BASE}.pgm ($(head -c 32 "${MAP_BASE}.pgm" | tr '\n' ' '))"
else
  SAVE_FAILED=1
  echo "!!! 保存地图失败: 检查 slam_toolbox 是否还在运行、/map 是否有发布者" >&2
fi

# 收尾: 先杀 launch, 再清掉它留下的 gzserver/gzclient/rviz2 子进程, 避免孤儿进程占端口
# 注意 set -e + pkill 的坑: pkill 没匹配到进程会返回 1, 会让脚本在清理途中直接退出,
# 后面的进程就杀不干净(实测会残留 gzserver/rviz2), 所以这里统一加 || true。
set +e
kill "$LAUNCH_PID" 2>/dev/null
sleep 2
pkill -9 -f "slam_demo.launch.py" 2>/dev/null || true
pkill -9 -f "[g]zserver" 2>/dev/null || true
pkill -9 -f "[g]zclient" 2>/dev/null || true
pkill -9 -f "[a]sync_slam_toolbox" 2>/dev/null || true
pkill -9 -f "[r]viz2" 2>/dev/null || true
set -e
if [ "$SAVE_FAILED" -ne 0 ]; then
  echo ">>> 清理完成, 但地图没有保存 (见上面的报错)" >&2
  exit 1
fi
echo ">>> 完成。建图结果: $(basename "$MAP_BASE").pgm"
echo "    导航演示(demo2/3/4)用的是与仿真世界一致的 maps/obstacle_field.pgm,"
echo "    若要改用本次建图结果, 手动把它拷到地图目录:"
echo "    cp \"$MAP_BASE\".pgm \"$MAP_BASE\".yaml $HOME/car_maps/  # 再重命名为 obstacle_field.*"
echo ">>> 同步到 Windows: scripts/sync_back.sh"
