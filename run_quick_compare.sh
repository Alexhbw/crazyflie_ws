#!/bin/bash
# 快速求解器对比 — 5 个求解器 × 固定步长 × 4 架飞机 × 障碍物开
# 每个跑 15 秒仿真, 输出对比 CSV

source install/setup.bash

DURATION=15.0
NUM_DRONES=4
STEP_SIZE=0.01
SOLVERS=("euler" "heun" "rk4" "rk45" "implicit")
OUTDIR="compare_results"
mkdir -p "$OUTDIR"
SUMMARY="${OUTDIR}/quick_compare.csv"

echo "📊 快速求解器对比 (${#SOLVERS[@]} 个求解器 × ${DURATION}s)"
echo "   步长: $STEP_SIZE | 飞机: $NUM_DRONES | 障碍物: ON"
echo "   ⏳ 运行中..."
echo ""

echo "solver,energy_drift_rate,min_obstacle_dist,max_velocity,max_attitude,rms_position_error,avg_compute_ms,collision_score" > "$SUMMARY"

for SOLVER in "${SOLVERS[@]}"; do
    TAG="${SOLVER}_dt${STEP_SIZE}_n${NUM_DRONES}"
    METRICS="${OUTDIR}/metrics_${TAG}.csv"
    SAFETY="${OUTDIR}/safety_${TAG}.csv"

    printf "  [%s] " "$SOLVER"
    ros2 run crazyflie_ode_core ode_engine_node --ros-args \
      -p solver:=$SOLVER \
      -p step_size:=$STEP_SIZE \
      -p num_drones:=$NUM_DRONES \
      -p enable_obstacles:=true \
      -p experiment.enabled:=true \
      -p experiment.duration:=$DURATION \
      -p experiment.headless:=true \
      -p experiment.metrics_path:=$METRICS \
      -p safety.log_path:=$SAFETY \
      -p safety.verbose:=false 2>/dev/null

    if [ -f "$METRICS" ]; then
        # metrics CSV 列: solver,step_size,...,energy_drift_rate(6),min_obstacle_dist(7),max_velocity(8),max_attitude(9),rms_pos_error(10),avg_compute_ms(11),collision(12),...
        DATA=$(tail -1 "$METRICS")
        DRIFT=$(echo "$DATA" | cut -d',' -f6)
        OBDIST=$(echo "$DATA" | cut -d',' -f7)
        VMAX=$(echo "$DATA" | cut -d',' -f8)
        AMAX=$(echo "$DATA" | cut -d',' -f9)
        RMS=$(echo "$DATA" | cut -d',' -f10)
        COMP=$(echo "$DATA" | cut -d',' -f11)
        COLSCORE=$(echo "$DATA" | cut -d',' -f12)
        echo "$SOLVER,$DRIFT,$OBDIST,$VMAX,$AMAX,$RMS,$COMP,$COLSCORE" >> "$SUMMARY"
        printf "✅\n"
    else
        echo "❌ (no metrics)"
    fi
done

echo ""
echo "=== 对比结果 ==="
column -t -s',' "$SUMMARY"
echo ""
echo "完整数据: $SUMMARY"
