#!/bin/bash
# 多步长对比 — 5 求解器 × 3 步长, 能看到差异
source install/setup.bash
OUTDIR="compare_results"
mkdir -p "$OUTDIR"
SUMMARY="${OUTDIR}/multi_dt_compare.csv"

SOLVERS=("euler" "heun" "rk4" "rk45" "implicit")
STEP_SIZES=(0.01 0.05 0.10)
NUM_DRONES=4
DURATION=15.0

echo "📊 多步长对比 (${#SOLVERS[@]} 求解器 × ${#STEP_SIZES[@]} 步长)"
echo "   飞机: $NUM_DRONES | 时长: ${DURATION}s | 障碍物: ON"
echo "   ⏳ 共 $(( ${#SOLVERS[@]} * ${#STEP_SIZES[@]} )) 次实验..."
echo ""

echo "solver,step_size,num_drones,obstacles_enabled,sim_duration,energy_drift_rate,min_obstacle_dist,max_velocity,max_attitude,rms_position_error,avg_compute_ms,collision_compliance,velocity_compliance,attitude_compliance" > "$SUMMARY"

for DT in "${STEP_SIZES[@]}"; do
    for SOLVER in "${SOLVERS[@]}"; do
        TAG="${SOLVER}_dt${DT}_n${NUM_DRONES}"
        METRICS="${OUTDIR}/metrics_${TAG}.csv"
        SAFETY="${OUTDIR}/safety_${TAG}.csv"

        printf "  [dt=%s %s] " "$DT" "$SOLVER"
        ros2 run crazyflie_ode_core ode_engine_node --ros-args \
          -p solver:=$SOLVER -p step_size:=$DT \
          -p num_drones:=$NUM_DRONES -p enable_obstacles:=true \
          -p experiment.enabled:=true -p experiment.duration:=$DURATION \
          -p experiment.headless:=true \
          -p experiment.metrics_path:=$METRICS \
          -p safety.log_path:=$SAFETY \
          -p safety.warmup_time:=0.0 \
          -p safety.verbose:=false 2>/dev/null

        if [ -f "$METRICS" ]; then
            tail -1 "$METRICS" >> "$SUMMARY"
            printf "✅\n"
        else
            # 记录失败行
            echo "$SOLVER,$DT,$NUM_DRONES,true,$DURATION,,,,,,,FAIL,FAIL,FAIL" >> "$SUMMARY"
            printf "❌\n"
        fi
    done
done

echo ""
echo "完整数据: $SUMMARY"
echo ""