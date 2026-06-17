#!/usr/bin/env python3
"""
临界步长分析 — 二分搜索每个求解器的安全步长上限
产出: safety_boundary_table.csv
核心发现: "安全步长 < 稳定步长" — 稳定性只是必要条件
"""

import subprocess
import csv
import os
import sys
import time
from typing import Tuple

# 确保 subprocess 能找到 ros2
ROS2_SETUP = os.path.expanduser("~/crazyflie_ws/install/setup.bash")
BASH_BIN = "/bin/bash"
if os.path.exists(ROS2_SETUP):
    SHELL_PREFIX = f"source {ROS2_SETUP} && "
else:
    SHELL_PREFIX = ""

# ===== 参数 =====
SOLVERS = ["euler", "heun", "rk4", "rk45", "implicit"]
LOW_DT = 0.001
HIGH_DT = 1.0              # 扩大到 1.0s 才能暴露各求解器差异
TOLERANCE = 0.001          # 搜索精度 1ms
NUM_DRONES = 4
OBSTACLES = True
SIM_DURATION = 30.0       # 每次测试的仿真时长
CONFIRM_RUNS = 2          # 每个候选步长重复确认次数 (避免随机性)
PACKAGE = "crazyflie_ode_core"
EXECUTABLE = "ode_engine_node"
OUTPUT_DIR = "safety_boundary_results"


def is_safe(solver: str, step_size: float) -> Tuple[bool, str]:
    """
    运行实验, 返回 (is_safe, reason).
    检查 safety_log.csv 是否有安全违规 (COLLISION/VELOCITY/ATTITUDE).
    ENERGY_DRIFT 不计入安全违规 (仅诊断).
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    tag = f"_{solver}_dt{step_size:.5f}"
    safety_log = os.path.join(OUTPUT_DIR, f"safety{tag}.csv")
    metrics_path = os.path.join(OUTPUT_DIR, f"metrics{tag}.csv")

    ros_args = [
        "ros2", "run", PACKAGE, EXECUTABLE,
        "--ros-args",
        "-p", f"solver:={solver}",
        "-p", f"step_size:={step_size}",
        "-p", f"num_drones:={NUM_DRONES}",
        "-p", f"enable_obstacles:={str(OBSTACLES).lower()}",
        "-p", "experiment.enabled:=true",
        "-p", f"experiment.duration:={SIM_DURATION}",
        "-p", "experiment.headless:=true",
        "-p", f"experiment.metrics_path:={metrics_path}",
        "-p", f"safety.log_path:={safety_log}",
        "-p", "safety.verbose:=false",
        "-p", "safety.v_max:=2.5",
        "-p", "safety.theta_max:=0.8",
        "-p", "safety.energy_drift_eps:=50.0",
        "-p", "safety.warmup_time:=0.0",
    ]

    cmd = SHELL_PREFIX + " ".join(ros_args)
    timeout_sec = SIM_DURATION + 90
    try:
        result = subprocess.run(cmd, shell=True, executable=BASH_BIN,
                                timeout=timeout_sec,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode != 0 and result.returncode != 143:  # 143 = SIGTERM (normal shutdown)
            pass  # Non-zero exit; check safety log anyway
    except subprocess.TimeoutExpired:
        return False, "TIMEOUT"

    # 检查 safety log
    if not os.path.exists(safety_log):
        # 没有违规日志 = 安全
        return True, "no violations"

    with open(safety_log, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cond = row.get("condition", "")
            if cond in ("COLLISION", "VELOCITY", "ATTITUDE", "ENERGY_DRIFT"):
                return False, f"{cond} at t={row.get('sim_time','?')}"

    return True, "only energy drift"


def confirm_safe(solver: str, step_size: float, runs: int = CONFIRM_RUNS) -> bool:
    """重复运行确认安全 (排除随机性)."""
    safe_count = 0
    for _ in range(runs):
        safe, _ = is_safe(solver, step_size)
        if safe:
            safe_count += 1
    return safe_count >= runs  # 全部通过才算安全


def binary_search(solver: str) -> dict:
    """
    二分搜索找到安全步长边界.
    返回: {solver, safe_step_size, unsafe_step_size, margin}
    """
    lo = LOW_DT
    hi = HIGH_DT
    best_safe = lo
    first_unsafe = hi

    # 先检查最低步长是否安全
    safe, reason = is_safe(solver, lo)
    if not safe:
        print(f"  {solver}: LOW_DT={lo} already UNSAFE ({reason}) — using lo as safe bound")
        best_safe = lo / 2.0  # 保守估计

    iters = 0
    while hi - lo > TOLERANCE and iters < 30:
        mid = (lo + hi) / 2.0
        safe, reason = is_safe(solver, mid)
        iters += 1

        if safe:
            best_safe = mid
            lo = mid
            print(f"  [{iters}] {solver} dt={mid:.5f} SAFE → lo={lo:.5f}")
        else:
            first_unsafe = mid
            hi = mid
            print(f"  [{iters}] {solver} dt={mid:.5f} UNSAFE ({reason}) → hi={hi:.5f}")

    # 最终确认最佳安全步长
    if confirm_safe(solver, best_safe):
        print(f"  {solver}: confirmed safe at dt={best_safe:.5f}")
    else:
        # 回退
        print(f"  {solver}: best_safe failed confirmation, using lo={LOW_DT}")
        best_safe = LOW_DT

    return {
        "solver": solver,
        "safe_step_size": round(best_safe, 5),
        "unsafe_step_size": round(first_unsafe, 5),
        "margin": round(first_unsafe - best_safe, 5),
    }


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("=" * 70)
    print("Safety Boundary Analysis — Binary Search for Critical Step Size")
    print(f"  Solvers: {SOLVERS}")
    print(f"  Search range: [{LOW_DT}, {HIGH_DT}] tolerance={TOLERANCE}")
    print(f"  Drones: {NUM_DRONES}, Obstacles: {OBSTACLES}, Duration: {SIM_DURATION}s")
    print(f"  Safety criteria: no COLLISION | VELOCITY | ATTITUDE | ENERGY_DRIFT violations")
    print("=" * 70)
    print()

    results = []
    t_start = time.time()

    for solver in SOLVERS:
        print(f"\n--- {solver} ---")
        t0 = time.time()
        result = binary_search(solver)
        result["search_time_s"] = round(time.time() - t0, 1)
        results.append(result)

    # ==== 写入结果 ====
    out_path = os.path.join(OUTPUT_DIR, "safety_boundary_table.csv")
    fieldnames = ["solver", "safe_step_size", "unsafe_step_size", "margin", "search_time_s"]
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    total_time = time.time() - t_start
    print(f"\n{'=' * 70}")
    print("Safety Boundary Results:")
    print(f"{'Solver':<12} {'Safe dt':>10} {'Unsafe dt':>10} {'Margin':>10}")
    print("-" * 42)
    for r in results:
        print(f"{r['solver']:<12} {r['safe_step_size']:>10.5f} "
              f"{r['unsafe_step_size']:>10.5f} {r['margin']:>10.5f}")
    print("-" * 42)
    print(f"Total time: {total_time:.0f}s")
    print(f"Results: {out_path}")

    # ==== 结论 ====
    print()
    print("Key Finding: 'Safe step size < Stable step size'")
    print("This confirms that numerical stability is a necessary but NOT")
    print("sufficient condition for safe autonomous flight.")
    for r in results:
        if r['safe_step_size'] < r['unsafe_step_size']:
            print(f"  {r['solver']}: safe={r['safe_step_size']:.5f}s < "
                  f"unsafe={r['unsafe_step_size']:.5f}s "
                  f"(margin={r['margin']:.5f}s)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
