#!/bin/bash

# 加载环境
source install/setup.bash

# 获取传入的参数
SOLVER=$1
STEP_SIZE=$2
# 获取第三个参数（无人机数量），如果不填，默认就是 1
NUM_DRONES=${3:-1}
# 获取第四个参数（是否启用障碍物），默认 true
ENABLE_OBS=${4:-true}

echo "🚀 开始测试 | 算法: $SOLVER | 步长: $STEP_SIZE | 编队数量: $NUM_DRONES 架 | 障碍物: $ENABLE_OBS"
RSP_PIDS=""

# RSP 和 RViz 同时启动 (原始竞态, 配合 Transient Local RViz 总能收到 robot_description)
for ((i=0; i<$NUM_DRONES; i++))
do
  ros2 run robot_state_publisher robot_state_publisher \
    /home/alex/crazyflie_ws/src/crazyflie_ode_core/urdf/crazyflie.urdf \
    --ros-args -r __ns:=/cf$i -p frame_prefix:=cf$i/ -p use_sim_time:=true > /dev/null 2>&1 &
  RSP_PIDS="$RSP_PIDS $!"
done

rviz2 -d /home/alex/crazyflie_ws/src/crazyflie_ode_core/rviz/swarm.rviz \
  --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
RVIZ_PID=$!

cleanup() {
    echo "🛑 清理..."
    for pid in $RSP_PIDS; do kill $pid 2>/dev/null; done
    kill $RVIZ_PID 2>/dev/null
    exit 0
}
trap cleanup INT TERM

ros2 run crazyflie_ode_core ode_engine_node --ros-args -p solver:=$SOLVER -p step_size:=$STEP_SIZE -p num_drones:=$NUM_DRONES -p enable_obstacles:=$ENABLE_OBS

cleanup
