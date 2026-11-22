#!/bin/bash
# ============================================================
# 官方标准链路一键管线 (slam_toolbox + Nav2 官方架构)
#   阶段A  slam_toolbox 建图 + roadmap_explorer 自主探索
#   存图   map_saver_cli (SLAM 还活着时保存)
#   切换   停 SLAM/Nav2/探索器/RViz, Gazebo 全程不重启
#   阶段B  map_server + AMCL 接管 map->odom + Nav2 航点巡逻
# 关键设计: 两个定位源绝不同时运行;
#           初始位姿 = slam map->odom TF ⊕ /odom 当前位姿 (机器人已不在出生点)
# 用法: bash run_official_pipeline.sh [最长探索秒数, 默认 1500]
# 环境变量: TARGET_COVERAGE=0.95 CHECK_INTERVAL=30 MAX_RESTARTS=8
# ============================================================
set -e
DURATION="${1:-1500}"
TARGET_COVERAGE="${TARGET_COVERAGE:-0.95}"
CHECK_INTERVAL="${CHECK_INTERVAL:-30}"
MAX_RESTARTS="${MAX_RESTARTS:-8}"
STALL_CHECKS_TO_RESTART=3
# 卡死挪车: 地图连续 2 个周期(2*CHECK_INTERVAL 秒)没更新就触发挪车
MAP_STALL_CHECKS_TO_RESTART=2
LAST_MAP_SIG=""
MAP_STALL_TICKS=0
BOUNDARY_HALF=5.8
# 挪车安全点: 出生点, 必然已知自由且可达(车从这里出发)
RECOVERY_X="${RECOVERY_X:--4.5}"
RECOVERY_Y="${RECOVERY_Y:--4.5}"
WS_CAR="/home/yzh/workspace/F盘/listudy/car/car_ws"
WS_ROADMAP="/home/yzh/roadmap_ws"
MAP_BASE="/home/yzh/car_maps/official_map"          # 官方链路产出的地图
GT_REF_DIR="$WS_CAR/src/car_nav/_vmfix/gt_map"      # gen_gt_map.py 参考图
COVERAGE_SCRIPT="$WS_CAR/src/car_nav/scripts/reachable_coverage.py"
SESSION="$(date +%Y%m%d_%H%M%S)_$$"
LOG_DIR="/tmp/official_pipeline_${SESSION}"
mkdir -p "$LOG_DIR"
GOAL_PID=""

# 显示设置 (VMware 桌面, 全程可视化)
export DISPLAY=:0
export XAUTHORITY=/home/yzh/.Xauthority
# CycloneDDS 必须, FastDDS 会 segfault
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$WS_ROADMAP/install/setup.bash"
source "$WS_CAR/install/setup.bash"
export TURTLEBOT3_MODEL=burger

cleanup_all() {
    set +e
    echo '==清理全部进程=='
    pkill -f '[r]oadmap_exploration_server' || true
    pkill -f '[r]oadmap_explorer' || true
    pkill -f '[r]oadmap_explore.launch.py' || true
    pkill -f '[s]lam_nav.launch.py' || true
    pkill -f '[l]ocalization_nav.launch.py' || true
    pkill -f '[a]sync_slam_toolbox' || true
    pkill -f '[f]rontier_explore' || true
    pkill -f '[m]ission_runner' || true
    pkill -f '[i]nitial_pose_publisher' || true
    pkill -f '[l]ifecycle_manager' || true
    pkill -f '[c]ontroller_server' || true
    pkill -f '[p]lanner_server' || true
    pkill -f '[b]ehavior_server' || true
    pkill -f '[s]moother_server' || true
    pkill -f '[v]elocity_smoother' || true
    pkill -f '[w]aypoint_follower' || true
    pkill -f '[b]t_navigator' || true
    pkill -f '[n]av2_map_server' || true
    pkill -f '[n]av2_amcl' || true
    pkill -f '[r]viz2' || true
    pkill -f '[s]tatic_transform_publisher' || true
    pkill -f '[g]zserver' || true
    pkill -f '[g]zclient' || true
    sleep 2
    pkill -9 -f '[g]zserver' || true
    pkill -9 -f '[g]zclient' || true
}
# 管线结束时保留 Gazebo+RViz 供查看; 只有被 Ctrl+C 时才全清
trap 'cleanup_all; exit 130' INT TERM

kill_slam_stack() {
    # 只停 SLAM/Nav2/探索器/RViz, 保留 Gazebo(gzserver/gzclient)与
    # robot_state_publisher, 机器人位姿在世界中连续 —— 官方链路切换关键
    set +e
    echo '==停 SLAM/Nav2/探索器 (Gazebo 保持运行)=='
    # 先杀阶段A launch 父进程 (SIGKILL 不触发子进程级联关闭)
    kill -9 "$LAUNCH_A_PID" 2>/dev/null || true
    pkill -f '[r]oadmap_exploration_server' || true
    pkill -f '[r]oadmap_explorer' || true
    pkill -f '[a]sync_slam_toolbox' || true
    pkill -f '[c]ontroller_server' || true
    pkill -f '[p]lanner_server' || true
    pkill -f '[b]ehavior_server' || true
    pkill -f '[s]moother_server' || true
    pkill -f '[v]elocity_smoother' || true
    pkill -f '[w]aypoint_follower' || true
    pkill -f '[b]t_navigator' || true
    pkill -f '[l]ifecycle_manager' || true
    pkill -f '[r]viz2' || true
    pkill -f '[s]tatic_transform_publisher' || true
    sleep 3
    # 防Nav2 节点残留占行为服务器端口
    pkill -9 -f '[a]sync_slam_toolbox' || true
    pkill -9 -f '[c]ontroller_server' || true
    pkill -9 -f '[p]lanner_server' || true
    pkill -9 -f '[b]t_navigator' || true
    if ! pgrep -x gzserver > /dev/null; then
        echo '==失败: Gazebo 意外退出, 无法无缝切换=='
        exit 3
    fi
    echo '==Gazebo 存活, 切换窗口就绪=='
    set -e
}

save_and_check_coverage() {
    local tmp_map="/tmp/official_cov_${SESSION}"
    rm -f "${tmp_map}.pgm" "${tmp_map}.yaml"
    if ! timeout 15 ros2 run nav2_map_server map_saver_cli -f "$tmp_map" \
        --ros-args -p use_sim_time:=true > /tmp/official_map_saver.log 2>&1; then
        echo '临时地图保存失败, 本次跳过覆盖率检查'
        return 2
    fi
    if [ ! -s "${tmp_map}.pgm" ] || [ ! -s "${tmp_map}.yaml" ]; then
        echo '临时地图文件不存在, 本次跳过覆盖率检查'
        return 2
    fi
    # 地图连续不更新也是停滞: Nav2 卡死时不一定打印无可达 frontier
    local sig
    sig=$(md5sum "${tmp_map}.pgm")
    sig=${sig%% *}
    if [ "$sig" = "$LAST_MAP_SIG" ]; then
        MAP_STALL_TICKS=$((MAP_STALL_TICKS + 1))
    else
        MAP_STALL_TICKS=0
    fi
    LAST_MAP_SIG=$sig
    python3 "$COVERAGE_SCRIPT" "${tmp_map}.pgm" "$BOUNDARY_HALF" \
        --reference "$REFERENCE_MAP" --target "$TARGET_COVERAGE"
}

start_goal() {
    local mode="$1"
    GOAL_LOG="$LOG_DIR/explore_goal_${RESTARTS}.log"
    if [ "$mode" -eq 0 ]; then
        echo '==发送新探索会话 action goal=='
    else
        echo "==续跑已终止会话 (第 $RESTARTS/$MAX_RESTARTS 次)=="
    fi
    ros2 action send_goal /roadmap_explorer roadmap_explorer_msgs/action/Explore \
        "{exploration_bringup_mode: $mode}" > "$GOAL_LOG" 2>&1 &
    GOAL_PID=$!
}

stop_goal() {
    if kill -0 "$GOAL_PID" 2>/dev/null; then
        kill -INT "$GOAL_PID" 2>/dev/null || true
        for ((i = 0; i < 10; i++)); do
            kill -0 "$GOAL_PID" 2>/dev/null || break
            sleep 1
        done
        kill -0 "$GOAL_PID" 2>/dev/null && return 1
    fi
    wait "$GOAL_PID" 2>/dev/null || true
    return 0
}

# 卡死挪车: 机器人钻进不可达前沿包围圈出不来时, 先取消探索,
# 用 Nav2 把车真实开回安全点(不瞬移, 避免撕裂 slam_toolbox 位姿图),
# 打破包围圈后再续跑。超时/失败则退回原来的直接续跑行为。
drive_back() {
    echo "==挪车: 开回安全点 ($RECOVERY_X, $RECOVERY_Y) 再续跑=="
    sleep 3   # 等探索 goal 取消后 bt_navigator 释放
    timeout 100 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $RECOVERY_X, y: $RECOVERY_Y, z: 0.0}, orientation: {w: 1.0}}}}" \
        > "/tmp/recovery_nav_${SESSION}.log" 2>&1
    if grep -q 'SUCCEEDED' "/tmp/recovery_nav_${SESSION}.log" 2>/dev/null; then
        echo '==挪车完成, 已回到安全点=='
    else
        echo '==挪车未确认成功(超时/不可达), 直接续跑=='
    fi
    sleep 2
}

# ---- GT 参考图: 与仿真世界几何完全一致 (gen_gt_map.py 离线生成) ----
echo '==准备 GT 参考图=='
if [ ! -f "$GT_REF_DIR/obstacle_field.pgm" ]; then
    (cd "$WS_CAR/src/car_nav" && python3 tests/gen_gt_map.py > "$LOG_DIR/gen_gt.log" 2>&1) || {
        echo 'gen_gt_map.py 失败'; exit 2; }
fi
REFERENCE_MAP="$GT_REF_DIR/obstacle_field.pgm"
echo "参考图: $REFERENCE_MAP"

# ---- 清理旧进程 (与 run_demo1_roadmap.sh 相同的清场纪律) ----
echo '==清理旧进程=='
cleanup_all || true
sleep 2

# ================= 阶段 A: SLAM 建图 + 探索 =================
echo '==[阶段A] 启动 slam_nav.launch.py (SLAM 提供定位)=='
ros2 launch car_nav slam_nav.launch.py > "$LOG_DIR/phase_a.log" 2>&1 &
LAUNCH_A_PID=$!
echo "Launch A PID: $LAUNCH_A_PID, 日志: $LOG_DIR/phase_a.log"

echo '==等 30 秒让 Gazebo/Nav2/SLAM 就绪=='
sleep 30
if ! timeout 10 ros2 topic echo --once /clock >/dev/null 2>&1; then
    echo '==启动失败: /clock 无数据=='; cleanup_all; exit 1
fi
if ! timeout 10 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo '==启动失败: /scan 无数据=='; cleanup_all; exit 1
fi
echo '==仿真数据流正常=='

RESTARTS=0
STALL_TICKS=0
LAST_STALLS=0
REACHED=0
REASON="达到最长探索时间 ${DURATION}s"
START_TIME=$SECONDS
LAST_CHECK=$SECONDS
start_goal 0

echo "==运行探索, 目标参考自由空间覆盖率 ${TARGET_COVERAGE}, 最多 $DURATION 秒=="
while ((SECONDS - START_TIME < DURATION)); do
    sleep 5
    kill -0 "$LAUNCH_A_PID" 2>/dev/null || { REASON='slam_nav launch 异常退出'; break; }

    if ((SECONDS - LAST_CHECK >= CHECK_INTERVAL)); then
        echo '==定期覆盖率检查=='
        if save_and_check_coverage; then
            REACHED=1; REASON='达到目标参考自由空间覆盖率'; break
        fi
        LAST_CHECK=$SECONDS
    fi

    CURRENT_STALLS=$(grep -Eci 'no achievable frontiers left|no frontiers in current search radius' "$LOG_DIR/phase_a.log" || true)
    if ((CURRENT_STALLS > LAST_STALLS)); then
        STALL_TICKS=$((STALL_TICKS + 1))
    else
        STALL_TICKS=0
    fi
    LAST_STALLS=$CURRENT_STALLS
    if ((MAP_STALL_TICKS >= MAP_STALL_CHECKS_TO_RESTART)); then
        echo "地图连续 $((MAP_STALL_CHECKS_TO_RESTART * CHECK_INTERVAL)) 秒未更新, 触发停滞续跑"
        STALL_TICKS=$STALL_CHECKS_TO_RESTART
    fi

    GOAL_ENDED=0
    kill -0 "$GOAL_PID" 2>/dev/null || GOAL_ENDED=1
    if ((GOAL_ENDED == 0 && STALL_TICKS < STALL_CHECKS_TO_RESTART)); then
        continue
    fi

    echo '==探索停止或持续无可达 frontier, 先检查覆盖率=='
    if save_and_check_coverage; then
        REACHED=1; REASON='达到目标参考自由空间覆盖率'; break
    fi
    if ((RESTARTS >= MAX_RESTARTS)); then
        REASON="已达到最大续跑次数 $MAX_RESTARTS"; break
    fi
    if ((GOAL_ENDED == 1)); then
        wait "$GOAL_PID" 2>/dev/null || true
        echo '当前 goal 已退出'
    else
        echo '探索持续停滞, 取消当前 goal'
        stop_goal || { REASON='goal 取消失败'; break; }
    fi
    # 卡死挪车: 地图停滞(不可达前沿包围)时先把车开回安全点再续跑
    if ((MAP_STALL_TICKS >= MAP_STALL_CHECKS_TO_RESTART || STALL_TICKS >= STALL_CHECKS_TO_RESTART)); then
        drive_back
    fi
    RESTARTS=$((RESTARTS + 1))
    sleep 5
    start_goal 1
    STALL_TICKS=0
    MAP_STALL_TICKS=0
    LAST_STALLS=$(grep -Eci 'no achievable frontiers left|no frontiers in current search radius' "$LOG_DIR/phase_a.log" || true)
done

# ---- SLAM 存活期间保存正式地图 (官方要求: 停 SLAM 前存图) ----
echo '==保存正式地图 (map_saver_cli)=='
if [ -f "${MAP_BASE}.pgm" ]; then
    mv "${MAP_BASE}.pgm" "${MAP_BASE}.pgm.prev" || true
    mv "${MAP_BASE}.yaml" "${MAP_BASE}.yaml.prev" || true
fi
if timeout 20 ros2 run nav2_map_server map_saver_cli -f "$MAP_BASE" \
    --ros-args -p use_sim_time:=true; then
    sed -i -e '/^mode:/d' "${MAP_BASE}.yaml"
    if python3 "$COVERAGE_SCRIPT" "${MAP_BASE}.pgm" "$BOUNDARY_HALF" \
        --reference "$REFERENCE_MAP" --target "$TARGET_COVERAGE"; then
        REACHED=1; REASON='达到目标参考自由空间覆盖率'
    fi
else
    REASON='正式地图保存失败'
    cleanup_all; exit 4
fi
echo "地图已保存: ${MAP_BASE}.pgm/.yaml (覆盖率结论: $REACHED)"
# ---- 自动传递: 同步到 demo2/3/4 消费的名字 (~/car_maps/obstacle_field) ----
mkdir -p /home/yzh/car_maps
cp -f "${MAP_BASE}.pgm" /home/yzh/car_maps/obstacle_field.pgm
sed -e "s|$(basename "${MAP_BASE}").pgm|obstacle_field.pgm|" \
    -e '/^mode:/d' "${MAP_BASE}.yaml" > /home/yzh/car_maps/obstacle_field.yaml
echo '已自动传递到 /home/yzh/car_maps/obstacle_field.{pgm,yaml}'

# ---- 官方切换: 抓 map->odom TF ⊕ /odom 位姿, 计算阶段B初始位姿 ----
echo '==计算阶段B初始位姿 (slam map->odom TF ⊕ /odom)=='
# tf2_echo 持续打印不退出, 正常路径就是被 timeout 杀掉(退出码 124),
# 因此不能以退出码判成败, 以输出文件内容为准
timeout 8 ros2 run tf2_ros tf2_echo map odom > "$LOG_DIR/tf_map_odom.txt" 2>&1 || true
if ! grep -q 'Translation' "$LOG_DIR/tf_map_odom.txt"; then
    echo '==失败: 读不到 slam map->odom TF=='
    kill_slam_stack; cleanup_all; exit 5
fi
POSE_FILE="/tmp/official_init_pose_${SESSION}.txt"
python3 - "$LOG_DIR/tf_map_odom.txt" "$POSE_FILE" << 'PYEOF'
import math, re, sys
import rclpy
from nav_msgs.msg import Odometry

tf_path, pose_path = sys.argv[1], sys.argv[2]
txt = open(tf_path).read()
tx = ty = th = 0.0
m = re.search(r'- Translation: \[([-0-9.e]+), ([-0-9.e]+), ([-0-9.e]+)\]', txt)
m2 = re.search(r'RPY \(radian\): \[([-0-9.e]+), ([-0-9.e]+), ([-0-9.e]+)\]', txt)
if m and m2:
    tx, ty = float(m.group(1)), float(m.group(2))
    th = float(m2.group(3))
else:
    print('WARN: tf2_echo 解析失败, 按 map=odom 处理')

rclpy.init()
node = rclpy.create_node('odom_snapshot')
got = {}

def cb(msg):
    p = msg.pose.pose
    q = p.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    got['v'] = (p.position.x, p.position.y, yaw)

node.create_subscription(Odometry, '/odom', cb, 10)
import time
deadline = time.time() + 12
while time.time() < deadline and 'v' not in got:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()

if 'v' in got:
    ox, oy, oyaw = got['v']
else:
    print('WARN: /odom 读取失败, 回退出生点 (-4.5, -4.5, 0)')
    ox, oy, oyaw = -4.5, -4.5, 0.0

mx = tx + math.cos(th) * ox - math.sin(th) * oy
my = ty + math.sin(th) * ox + math.cos(th) * oy
myaw = math.atan2(math.sin(th + oyaw), math.cos(th + oyaw))
qz = math.sin(myaw / 2.0)
qw = math.cos(myaw / 2.0)
with open(pose_path, 'w') as f:
    f.write(f'{mx:.4f} {my:.4f} {myaw:.4f} {qz:.6f} {qw:.6f}\n')
print(f'map 初始位姿: x={mx:.3f} y={my:.3f} yaw={myaw:.3f}')
PYEOF

if [ ! -s "$POSE_FILE" ]; then
    echo '==失败: 初始位姿计算失败=='
    kill_slam_stack; cleanup_all; exit 5
fi
read MX MY MYAW QZ QW < "$POSE_FILE"
echo "阶段B初始位姿: ($MX, $MY, yaw=$MYAW)"

# ---- 停 SLAM, Gazebo 全程不重启 (官方链路切换关键) ----
kill_slam_stack

# ================= 阶段 B: map_server + AMCL 接管 + 航点巡逻 =================
echo '==[阶段B] 启动 localization_nav.launch.py (AMCL 提供定位)=='
ros2 launch car_nav localization_nav.launch.py \
    map:="${MAP_BASE}.yaml" \
    initial_x:="$MX" initial_y:="$MY" initial_yaw:="$MYAW" \
    run_mission:=true > "$LOG_DIR/phase_b.log" 2>&1 &
LAUNCH_B_PID=$!
echo "Launch B PID: $LAUNCH_B_PID, 日志: $LOG_DIR/phase_b.log"
sleep 40

# 备用播种: 再发一次 /initialpose (与 launch 内 initial_pose_publisher 同值, 无害)
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $MX, y: $MY, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" >/dev/null 2>&1 || true

# AMCL 接管验证
AMCL_OK=0
for i in $(seq 1 6); do
    if timeout 8 ros2 topic echo --once /amcl_pose >/dev/null 2>&1; then
        AMCL_OK=1; break
    fi
    sleep 5
done
if [ "$AMCL_OK" -ne 1 ]; then
    echo '==失败: AMCL 60 秒内未发布 /amcl_pose=='
    cleanup_all; exit 6
fi
echo '==AMCL 已接管 map->odom=='

# 等待 5 个航点巡逻完成 (mission_runner 逐点打印 "✓ 航点 i/5 到达")
MISSION_TIMEOUT=900
T0=$SECONDS
MISSION_OK=0
while ((SECONDS - T0 < MISSION_TIMEOUT)); do
    sleep 10
    if grep -q '航点 5/5 到达' "$LOG_DIR/phase_b.log"; then
        MISSION_OK=1; break
    fi
    if grep -Eq 'CANCELED|TaskResult.FAILED|ABORTED' "$LOG_DIR/phase_b.log"; then
        echo '==巡逻任务异常中断=='
        grep -E 'CANCELED|TaskResult.FAILED|ABORTED' "$LOG_DIR/phase_b.log" | tail -3
        break
    fi
    if ! kill -0 "$LAUNCH_B_PID" 2>/dev/null; then
        echo '==阶段B launch 异常退出=='
        break
    fi
done

echo '================== 官方链路管线结束 =================='
echo "阶段A探索: $REASON"
echo "产出地图 : ${MAP_BASE}.pgm/.yaml"
if [ "$MISSION_OK" -eq 1 ]; then
    echo '阶段B巡逻: 5/5 航点全部到达 ✓'
    echo "日志目录 : $LOG_DIR"
    echo '==Gazebo + RViz 保持运行, 可在桌面直接查看=='
    exit 0
fi
echo '阶段B巡逻: 未完成 (详见 phase_b.log)'
echo "日志目录 : $LOG_DIR"
# 失败也保留查看窗口: kill -9 不触发子进程级联, gzserver/gzclient/rviz2 存活
kill -9 "$LAUNCH_B_PID" 2>/dev/null || true
sleep 1
pkill -f '[m]ission_runner' || true
pkill -f '[b]t_navigator' || true
pkill -f '[c]ontroller_server' || true
pkill -f '[p]lanner_server' || true
pkill -f '[b]ehavior_server' || true
pkill -f '[n]av2_amcl' || true
pkill -f '[n]av2_map_server' || true
pkill -f '[s]tatic_transform_publisher' || true
echo '==Gazebo + RViz 保持运行, 可在桌面直接查看现场状态=='
exit 7
