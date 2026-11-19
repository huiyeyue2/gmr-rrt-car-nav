#!/bin/bash
# 基于 roadmap_explorer 的演示1自主探索建图
# 用法: bash run_demo1_roadmap.sh [最长运行秒数, 默认 1200]
# 可用环境变量: TARGET_COVERAGE=0.95 CHECK_INTERVAL=30 MAX_RESTARTS=8
# 注意: 必须用 CycloneDDS, FastDDS 会 segfault

set -e
DURATION="${1:-1200}"
TARGET_COVERAGE="${TARGET_COVERAGE:-0.95}"
CHECK_INTERVAL="${CHECK_INTERVAL:-30}"
MAX_RESTARTS="${MAX_RESTARTS:-8}"
STALL_CHECKS_TO_RESTART=3
MAP_STALL_CHECKS_TO_RESTART=4
LAST_MAP_SIG=""
MAP_STALL_TICKS=0
BOUNDARY_HALF=5.8
WS_CAR="/home/yzh/workspace/F盘/listudy/car/car_ws"
WS_ROADMAP="/home/yzh/roadmap_ws"
MAP_NAME="$WS_CAR/maps/obstacle_field_roadmap"
COVERAGE_SCRIPT="$WS_CAR/src/car_nav/scripts/reachable_coverage.py"
REFERENCE_MAP="$WS_CAR/src/car_nav/maps/obstacle_field.pgm"
SESSION="$(date +%Y%m%d_%H%M%S)_$$"
LOG_FILE="/tmp/roadmap_demo_${SESSION}.log"
COVERAGE_MAP="/tmp/roadmap_coverage_${SESSION}"
GOAL_PID=""

# 显示设置 (VMware 桌面)
export DISPLAY=:0
export XAUTHORITY=/home/yzh/.Xauthority

# CycloneDDS 必须, 不然 roadmap_explorer 会 segfault
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS + 两个 workspace
source /opt/ros/humble/setup.bash
source "$WS_ROADMAP/install/setup.bash"
source "$WS_CAR/install/setup.bash"
export TURTLEBOT3_MODEL=burger

cleanup() {
    set +e
    echo '==清理进程=='
    pkill -f '[r]oadmap_explorer roadmap_explorer_msgs/action/Explore'
    pkill -f '[r]oadmap_explore.launch.py'
    if [ -n "$GOAL_PID" ]; then
        kill -INT "$GOAL_PID" 2>/dev/null
    fi
    if [ -n "${LAUNCH_PID:-}" ]; then
        kill -INT "$LAUNCH_PID" 2>/dev/null
    fi
    sleep 3
    pkill -f '[g]zserver'
    pkill -f '[g]zclient'
    pkill -f '[r]viz2'
    pkill -f '[r]oadmap_exploration_server'
    # launch 被 SIGINT 后可能留下子节点: 不清会占着 /map 和 TF, 下次 RViz 只看到空白
    pkill -f '[a]sync_slam_toolbox_node'
    # 查看窗口遗留的静态 TF 会和 slam 的 map->odom 冲突, 必须一并清掉
    pkill -f '[s]tatic_transform_publisher'
    pkill -f '[n]av2_map_server'
    pkill -f '[c]ontroller_server'
    pkill -f '[p]lanner_server'
    pkill -f '[b]ehavior_server'
    pkill -f '[s]moother_server'
    pkill -f '[v]elocity_smoother'
    pkill -f '[w]aypoint_follower'
    pkill -f '[b]t_navigator'
    rm -f "${COVERAGE_MAP}.pgm" "${COVERAGE_MAP}.yaml"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 清理旧进程 (用括号模式防自匹配)
# 必须先停旧 launch/action: 只杀 gzserver 会留下旧仿真与重复探索会话
echo '==清理旧进程=='
pkill -f '[r]oadmap_explorer roadmap_explorer_msgs/action/Explore' || true
pkill -f '[r]oadmap_explore.launch.py' || true
pkill -f '[g]zserver' || true
pkill -f '[g]zclient' || true
pkill -f '[r]viz2' || true
pkill -f '[r]oadmap_exploration_server' || true
pkill -f '[f]rontier_explore' || true
pkill -f '[s]lam_toolbox' || true
pkill -f '[n]av2' || true
sleep 2
# 老 gzserver 偶尔不响应 SIGTERM, 强制清掉, 防止新实例因端口冲突退出
pkill -9 -f '[g]zserver' || true
pkill -9 -f '[g]zclient' || true

mkdir -p "$WS_CAR/maps"
for required_file in "$COVERAGE_SCRIPT" "$REFERENCE_MAP" "${REFERENCE_MAP%.pgm}.yaml"; do
    if [ ! -f "$required_file" ]; then
        echo "找不到覆盖率计算所需文件: $required_file"
        exit 2
    fi
done

start_goal() {
    local mode="$1"
    GOAL_LOG="/tmp/explore_goal_${SESSION}_${RESTARTS}.log"
    if [ "$mode" -eq 0 ]; then
        echo '==发送新探索会话 action goal=='
    else
        echo "==续跑已终止会话 (第 $RESTARTS/$MAX_RESTARTS 次)=="
    fi
    ros2 action send_goal /roadmap_explorer roadmap_explorer_msgs/action/Explore \
        "{exploration_bringup_mode: $mode}" > "$GOAL_LOG" 2>&1 &
    GOAL_PID=$!
    echo "Goal PID: $GOAL_PID, 日志: $GOAL_LOG"
}

save_and_check_coverage() {
    rm -f "${COVERAGE_MAP}.pgm" "${COVERAGE_MAP}.yaml"
    if ! timeout 15 ros2 run nav2_map_server map_saver_cli -f "$COVERAGE_MAP" \
        --ros-args -p use_sim_time:=true > /tmp/roadmap_map_saver.log 2>&1; then
        echo '临时地图保存失败, 本次跳过覆盖率检查'
        return 2
    fi
    if [ ! -s "${COVERAGE_MAP}.pgm" ] || [ ! -s "${COVERAGE_MAP}.yaml" ]; then
        echo '临时地图文件不存在, 本次跳过覆盖率检查'
        return 2
    fi
    # 地图连续不更新也是停滞: Nav2 卡死时不一定会打印无可达 frontier
    CURRENT_MAP_SIG=$(md5sum "${COVERAGE_MAP}.pgm")
    CURRENT_MAP_SIG=${CURRENT_MAP_SIG%% *}
    if [ "$CURRENT_MAP_SIG" = "$LAST_MAP_SIG" ]; then
        MAP_STALL_TICKS=$((MAP_STALL_TICKS + 1))
    else
        MAP_STALL_TICKS=0
    fi
    LAST_MAP_SIG=$CURRENT_MAP_SIG
    python3 "$COVERAGE_SCRIPT" "${COVERAGE_MAP}.pgm" "$BOUNDARY_HALF" \
        --reference "$REFERENCE_MAP" --target "$TARGET_COVERAGE"
}

stop_goal() {
    if kill -0 "$GOAL_PID" 2>/dev/null; then
        kill -INT "$GOAL_PID" 2>/dev/null || true
        for ((i = 0; i < 10; i++)); do
            if ! kill -0 "$GOAL_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
    fi
    if kill -0 "$GOAL_PID" 2>/dev/null; then
        echo '当前 goal 取消超时, 不发送并发 goal'
        return 1
    fi
    wait "$GOAL_PID" 2>/dev/null || true
}

restart_goal() {
    RESTARTS=$((RESTARTS + 1))
    sleep 5
    start_goal 1
    STALL_TICKS=0
    MAP_STALL_TICKS=0
    LAST_STALLS=$(grep -Eci 'no achievable frontiers left|no frontiers in current search radius' "$LOG_FILE" || true)
}

echo "==启动 roadmap_explore launch, 日志: $LOG_FILE=="
ros2 launch car_nav roadmap_explore.launch.py > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

echo '==等 30 秒让 Gazebo/Nav2/SLAM 就绪=='
sleep 30

# launch 进程存活不代表仿真可用; gzserver 死亡时 launch 仍可能空转
if ! timeout 10 ros2 topic echo --once /clock >/dev/null 2>&1; then
    echo '==启动失败: /clock 无数据, 停止无效测试=='
    exit 1
fi
if ! timeout 10 ros2 topic echo --once /scan >/dev/null 2>&1; then
    echo '==启动失败: /scan 无数据, 停止无效测试=='
    exit 1
fi

echo '==仿真数据流正常=='
RESTARTS=0
STALL_TICKS=0
LAST_STALLS=0
REACHED=0
REASON="达到最长运行时间 ${DURATION}s"
START_TIME=$SECONDS
LAST_CHECK=$SECONDS
start_goal 0

echo "==运行探索, 目标参考自由空间覆盖率 ${TARGET_COVERAGE}, 最多 $DURATION 秒=="
while ((SECONDS - START_TIME < DURATION)); do
    sleep 5
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        REASON='roadmap launch 异常退出'
        break
    fi

    if ((SECONDS - LAST_CHECK >= CHECK_INTERVAL)); then
        echo '==定期覆盖率检查=='
        if save_and_check_coverage; then
            REACHED=1
            REASON='达到目标参考自由空间覆盖率'
            break
        else
            CHECK_RC=$?
            if [ "$CHECK_RC" -eq 2 ]; then
                echo '覆盖率检查失败, 继续探索'
            fi
        fi
        LAST_CHECK=$SECONDS
    fi

    CURRENT_STALLS=$(grep -Eci 'no achievable frontiers left|no frontiers in current search radius' "$LOG_FILE" || true)
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
    if ! kill -0 "$GOAL_PID" 2>/dev/null; then
        GOAL_ENDED=1
    fi
    if ((GOAL_ENDED == 0 && STALL_TICKS < STALL_CHECKS_TO_RESTART)); then
        continue
    fi

    echo '==探索停止或持续无可达 frontier, 先检查覆盖率=='
    if save_and_check_coverage; then
        REACHED=1
        REASON='达到目标参考自由空间覆盖率'
        break
    fi
    if ((RESTARTS >= MAX_RESTARTS)); then
        REASON="已达到最大续跑次数 $MAX_RESTARTS"
        break
    fi

    if ((GOAL_ENDED == 1)); then
        wait "$GOAL_PID" 2>/dev/null || true
        echo '当前 goal 已退出'
    else
        echo '探索持续停滞, 取消当前 goal'
        if ! stop_goal; then
            REASON='goal 取消失败'
            break
        fi
    fi
    restart_goal
done

echo '==保存正式地图=='
if [ -f "${MAP_NAME}.pgm" ]; then
    mv "${MAP_NAME}.pgm" "${MAP_NAME}.pgm.prev" || true
    mv "${MAP_NAME}.yaml" "${MAP_NAME}.yaml.prev" || true
fi
if timeout 15 ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" \
    --ros-args -p use_sim_time:=true; then
    if python3 "$COVERAGE_SCRIPT" "${MAP_NAME}.pgm" "$BOUNDARY_HALF" \
        --reference "$REFERENCE_MAP" --target "$TARGET_COVERAGE"; then
        REACHED=1
        REASON='达到目标参考自由空间覆盖率'
    fi
else
    REASON='正式地图保存失败'
fi

echo "==结束: $REASON=="
echo "日志: $LOG_FILE"
echo "地图: $MAP_NAME"
if ((REACHED == 1)); then
    echo '==目标参考自由空间覆盖已完成, 自动停止=='
    exit 0
fi

echo '==未达到目标覆盖率, 已保存当前地图=='
exit 1
