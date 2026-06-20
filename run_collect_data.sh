#!/bin/bash
# 实验数据采集模式 (无 RViz, 自动退出, 输出 CSV)
# 用法: ./run_collect_data.sh <求解器> <步长> <飞机数> <时长秒> [障碍物开关]

source install/setup.bash

SOLVER=${1:-rk4}
STEP_SIZE=${2:-0.01}
NUM_DRONES=${3:-4}
DURATION=${4:-30.0}
# 确保 duration 带小数点 (ROS 2 double 参数不接受纯整数)
[[ "$DURATION" != *.* ]] && DURATION="${DURATION}.0"
ENABLE_OBS=${5:-true}
OUTDIR="experiment_results"
mkdir -p "$OUTDIR"

TAG="${SOLVER}_dt${STEP_SIZE}_n${NUM_DRONES}_obs${ENABLE_OBS}"
METRICS="${OUTDIR}/metrics_${TAG}.csv"
SAFETY="${OUTDIR}/safety_${TAG}.csv"

echo "📊 数据采集模式"
echo "   求解器: $SOLVER | 步长: $STEP_SIZE | 飞机: $NUM_DRONES"
echo "   时长: ${DURATION}s | 障碍物: $ENABLE_OBS"
echo "   输出: $METRICS"
echo "   ⏳ 运行中 (无 GUI)..."

ros2 run crazyflie_ode_core ode_engine_node --ros-args \
  -p solver:=$SOLVER \
  -p step_size:=$STEP_SIZE \
  -p num_drones:=$NUM_DRONES \
  -p enable_obstacles:=$ENABLE_OBS \
  -p experiment.enabled:=true \
  -p experiment.duration:=$DURATION \
  -p experiment.headless:=true \
  -p experiment.metrics_path:=$METRICS \
  -p safety.log_path:=$SAFETY \
  -p safety.verbose:=false 2>&1

echo ""
echo "✅ 完成. 查看数据:"
echo "   指标: cat $METRICS"
echo "   违规: cat $SAFETY"
cat "$METRICS"
