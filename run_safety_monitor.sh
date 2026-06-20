#!/bin/bash
# 带安全监视器的正常仿真 (RViz 可见)
# 用法: ./run_safety_monitor.sh <求解器> <步长> <飞机数> [障碍物开关]

source install/setup.bash

SOLVER=${1:-rk4}
STEP_SIZE=${2:-0.01}
NUM_DRONES=${3:-4}
ENABLE_OBS=${4:-true}

echo "🔒 安全监视模式 | 求解器: $SOLVER | 步长: $STEP_SIZE | 飞机: $NUM_DRONES | 障碍物: $ENABLE_OBS"

RSP_PIDS=""
for ((i=0; i<$NUM_DRONES; i++)); do
  ros2 run robot_state_publisher robot_state_publisher \
    /home/alex/crazyflie_ws/src/crazyflie_ode_core/urdf/crazyflie.urdf \
    --ros-args -r __ns:=/cf$i -p frame_prefix:=cf$i/ -p use_sim_time:=true > /dev/null 2>&1 &
  RSP_PIDS="$RSP_PIDS $!"
done

rviz2 -d /home/alex/crazyflie_ws/src/crazyflie_ode_core/rviz/swarm.rviz \
  --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
RVIZ_PID=$!
RVIZ_PID=$!

cleanup() {
    echo "🛑 清理..."
    for pid in $RSP_PIDS; do kill $pid 2>/dev/null; done
    kill $RVIZ_PID 2>/dev/null
    exit 0
}
trap cleanup INT TERM

ros2 run crazyflie_ode_core ode_engine_node --ros-args \
  -p solver:=$SOLVER \
  -p step_size:=$STEP_SIZE \
  -p num_drones:=$NUM_DRONES \
  -p enable_obstacles:=$ENABLE_OBS \
  -p safety.verbose:=true

cleanup
