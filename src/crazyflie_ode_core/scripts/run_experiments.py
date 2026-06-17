#!/usr/bin/env python3
"""
求解器稳定域对比实验框架
对 solver x step_size x num_drones x obstacles 做全因子扫描,
收集安全指标并汇总到 comparison_results.csv
"""

import subprocess
import csv
import os
import sys
import time
from itertools import product

ROS2_SETUP = os.path.expanduser("~/crazyflie_ws/install/setup.bash")
BASH_BIN = "/bin/bash"
SHELL_PREFIX = f"source {ROS2_SETUP} && " if os.path.exists(ROS2_SETUP) else ""

# ===== 扫描参数矩阵 =====
SOLVERS = ["euler", "heun", "rk4", "rk45", "implicit"]
STEP_SIZES = [0.001, 0.005, 0.01, 0.02, 0.05]
NUM_DRONES_VALUES = [1, 4, 8]
OBSTACLE_VALUES = [True, False]
SIM_DURATION = 30.0

PACKAGE = "crazyflie_ode_core"
EXECUTABLE = "ode_engine_node"
OUTPUT_DIR = "experiment_results"


def run_single(solver: str, step_size: float, num_drones: int,
               obstacles: bool, duration: float = SIM_DURATION) -> str:
    """
    运行单次实验, 返回 metrics CSV 路径.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    tag = f"{solver}_dt{step_size}_n{num_drones}_obs{str(obstacles).lower()}"
    safety_log = os.path.join(OUTPUT_DIR, f"safety_{tag}.csv")
    metrics_path = os.path.join(OUTPUT_DIR, f"metrics_{tag}.csv")

    ros_args = [
        "ros2", "run", PACKAGE, EXECUTABLE,
        "--ros-args",
        "-p", f"solver:={solver}",
        "-p", f"step_size:={step_size}",
        "-p", f"num_drones:={num_drones}",
        "-p", f"enable_obstacles:={str(obstacles).lower()}",
        "-p", "experiment.enabled:=true",
        "-p", f"experiment.duration:={duration}",
        "-p", "experiment.headless:=true",
        "-p", f"experiment.metrics_path:={metrics_path}",
        "-p", f"safety.log_path:={safety_log}",
        "-p", "safety.verbose:=false",
    ]

    timeout_sec = duration + 60
    cmd = SHELL_PREFIX + " ".join(ros_args)
    try:
        subprocess.run(cmd, shell=True, executable=BASH_BIN, timeout=timeout_sec,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        print(f"  [TIMEOUT] {tag} after {timeout_sec}s", file=sys.stderr)
        return ""

    if not os.path.exists(metrics_path):
        print(f"  [MISSING] {tag} — no metrics CSV produced", file=sys.stderr)
        return ""

    return metrics_path


def parse_metrics(csv_path: str) -> dict:
    """解析单次实验的指标 CSV, 返回 dict."""
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            return dict(row)
    return {}


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    total = len(SOLVERS) * len(STEP_SIZES) * len(NUM_DRONES_VALUES) * len(OBSTACLE_VALUES)
    print(f"=== Solver Stability Comparison Framework ===")
    print(f"    Solvers: {SOLVERS}")
    print(f"    Step sizes: {STEP_SIZES}")
    print(f"    Drone counts: {NUM_DRONES_VALUES}")
    print(f"    Obstacles: {OBSTACLE_VALUES}")
    print(f"    Duration per run: {SIM_DURATION}s")
    print(f"    Total runs: {total}")
    print(f"    Output dir: {OUTPUT_DIR}/")
    print()

    all_rows = []
    count = 0
    t_start = time.time()

    for solver, dt, n, obs in product(SOLVERS, STEP_SIZES,
                                       NUM_DRONES_VALUES, OBSTACLE_VALUES):
        count += 1
        tag = f"{solver} dt={dt} n={n} obs={obs}"
        elapsed = time.time() - t_start
        eta = (elapsed / count) * (total - count) if count > 0 else 0
        print(f"[{count}/{total}] {tag}  (elapsed={elapsed:.0f}s, ETA={eta:.0f}s)")

        csv_path = run_single(solver, dt, n, obs)
        if not csv_path:
            continue

        metrics = parse_metrics(csv_path)
        if not metrics:
            continue

        # 附加扫描参数
        metrics["sweep_solver"] = solver
        metrics["sweep_step_size"] = str(dt)
        metrics["sweep_num_drones"] = str(n)
        metrics["sweep_obstacles"] = str(obs)
        all_rows.append(metrics)

    # ==== 写入汇总 CSV ====
    if not all_rows:
        print("No results collected. Aborting.", file=sys.stderr)
        return 1

    out_path = os.path.join(OUTPUT_DIR, "comparison_results.csv")
    fieldnames = [
        "sweep_solver", "sweep_step_size", "sweep_num_drones", "sweep_obstacles",
        "energy_drift_rate", "min_obstacle_dist", "max_velocity", "max_attitude",
        "rms_position_error", "avg_compute_ms",
        "collision_compliance", "velocity_compliance", "attitude_compliance"
    ]

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(all_rows)

    total_time = time.time() - t_start
    print(f"\n=== Done ===")
    print(f"    Collected {len(all_rows)}/{total} results")
    print(f"    Total time: {total_time:.0f}s")
    print(f"    Aggregate results: {out_path}")

    # ==== 打印简表 ====
    print()
    print("Quick Summary (min_obstacle_dist per solver x step_size, n=4, obs=true):")
    print(f"{'solver':<10}", end="")
    for dt in STEP_SIZES:
        print(f"  dt={dt:<6}", end="")
    print()
    for solver in SOLVERS:
        print(f"{solver:<10}", end="")
        for dt in STEP_SIZES:
            dist = "N/A"
            for row in all_rows:
                if (row.get("sweep_solver") == solver and
                    row.get("sweep_step_size") == str(dt) and
                    row.get("sweep_num_drones") == "4" and
                    row.get("sweep_obstacles") == "True"):
                    dist = f"{float(row.get('min_obstacle_dist', 0)):.3f}"
                    break
            print(f"  {dist:<8}", end="")
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
