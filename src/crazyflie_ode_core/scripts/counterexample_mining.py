#!/usr/bin/env python3
"""
反例自动挖掘 — 搜索"A 炸了但 B 安全"的最小差异场景
对每个求解器找到临界步长，在此步长测试所有其他求解器
产出: counterexample_results/ 下的交叉对比表和热力图
"""

import subprocess
import csv
import os
import sys
import time
import json

ROS2_SETUP = os.path.expanduser("~/crazyflie_ws/install/setup.bash")
BASH_BIN = "/bin/bash"
SHELL_PREFIX = f"source {ROS2_SETUP} && " if os.path.exists(ROS2_SETUP) else ""

PACKAGE = "crazyflie_ode_core"
EXECUTABLE = "ode_engine_node"
OUTDIR = "counterexample_results"
NUM_DRONES = 4
OBSTACLES = True
SIM_DURATION = 10.0
LOW_DT = 0.005
HIGH_DT = 1.0
TOLERANCE = 0.001
SOLVERS = ["euler", "heun", "rk4", "rk45", "implicit"]
SOLVER_NAMES = {"euler":"Euler","heun":"Heun","rk4":"RK4","rk45":"RK45","implicit":"Implicit"}

# 测试步长网格 (log spacing, 15个点)
DT_GRID = [0.01, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00, 1.50, 2.00]


def run_experiment(solver, dt) -> float:
    """运行一次实验，返回 overall_score (1.0=完美, 0.0=崩溃)"""
    os.makedirs(OUTDIR, exist_ok=True)
    tag = f"mine_{solver}_dt{dt:.4f}"
    mpath = os.path.join(OUTDIR, f"m_{tag}.csv")
    spath = os.path.join(OUTDIR, f"s_{tag}.csv")

    ros_args = [
        "ros2", "run", PACKAGE, EXECUTABLE, "--ros-args",
        "-p", f"solver:={solver}",
        "-p", f"step_size:={dt}",
        "-p", f"num_drones:={NUM_DRONES}",
        "-p", f"enable_obstacles:={str(OBSTACLES).lower()}",
        "-p", "experiment.enabled:=true",
        "-p", f"experiment.duration:={SIM_DURATION}",
        "-p", "experiment.headless:=true",
        "-p", f"experiment.metrics_path:={mpath}",
        "-p", f"safety.log_path:={spath}",
        "-p", "safety.warmup_time:=0.0",
        "-p", "safety.energy_drift_eps:=10000.0",
        "-p", "safety.v_max:=100.0",
        "-p", "safety.theta_max:=100.0",
        "-p", "safety.verbose:=false",
    ]
    cmd = SHELL_PREFIX + " ".join(ros_args)
    timeout = SIM_DURATION + 60
    try:
        subprocess.run(cmd, shell=True, executable=BASH_BIN, timeout=timeout,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        return 0.0

    if not os.path.exists(mpath):
        return 0.0  # 实验根本没跑完

    # 从 metrics CSV 读碰撞/速度/姿态合规率
    try:
        with open(mpath) as f:
            for row in csv.DictReader(f):
                col = parse_float(row, "collision_compliance", 1.0)
                vel = parse_float(row, "velocity_compliance", 1.0)
                att = parse_float(row, "attitude_compliance", 1.0)
                return min(col, vel, att)
    except:
        pass
    return 0.0


def parse_float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except:
        return default


def find_critical_dt(solver):
    """二分搜索该求解器的临界步长"""
    lo, hi = LOW_DT, HIGH_DT
    best_safe = lo
    iters = 0
    while hi - lo > TOLERANCE and iters < 20:
        mid = (lo + hi) / 2.0
        score = run_experiment(solver, mid)
        iters += 1
        if score > 0.8:
            best_safe = mid
            lo = mid
            print(f"  [{iters}] {solver} dt={mid:.4f} SAFE (score={score:.3f})")
        else:
            hi = mid
            print(f"  [{iters}] {solver} dt={mid:.4f} UNSAFE (score={score:.3f})")
    return best_safe


def mine_grid():
    """在步长网格上跑所有求解器，生成热力图数据"""
    results = {}
    total = len(SOLVERS) * len(DT_GRID)
    count = 0
    t0 = time.time()

    print(f"Mining {len(SOLVERS)} solvers × {len(DT_GRID)} step sizes = {total} runs")
    print(f"Duration per run: {SIM_DURATION}s, est. total: {total * SIM_DURATION / 60:.0f} min\n")

    for dt in DT_GRID:
        results[dt] = {}
        for solver in SOLVERS:
            count += 1
            elapsed = time.time() - t0
            eta = (elapsed / count) * (total - count) if count > 0 else 0
            print(f"[{count}/{total}] dt={dt:.3f} {solver:10s}  ETA={eta/60:.0f}min", end=" ", flush=True)
            score = run_experiment(solver, dt)
            results[dt][solver] = score
            status = "SAFE" if score > 0.8 else ("WARN" if score > 0.3 else "CRASH")
            print(f"-> {score:.3f} {status}")

    total_time = time.time() - t0
    print(f"\nDone in {total_time/60:.1f} minutes")

    return results


def write_grid_csv(results):
    """写出热力图 CSV"""
    path = os.path.join(OUTDIR, "solver_safety_grid.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["step_size"] + SOLVERS
        writer.writerow(header)
        for dt in sorted(results.keys()):
            row = [f"{dt:.4f}"]
            for s in SOLVERS:
                row.append(f"{results[dt].get(s, 0.0):.4f}")
            writer.writerow(row)
    print(f"Grid CSV: {path}")
    return path


def find_counterexamples(results):
    """从网格数据中挖出反例: 同一 dt 下 A 安全 B 不安全"""
    examples = []
    for dt in sorted(results.keys()):
        scores = results[dt]
        safe_set = {s for s in SOLVERS if scores.get(s, 0) > 0.8}
        unsafe_set = {s for s in SOLVERS if scores.get(s, 0) <= 0.3}
        crash_set = {s for s in SOLVERS if scores.get(s, 0) <= 0.1}

        # 同一 dt: 有安全有崩溃 → 反例!
        if safe_set and crash_set:
            for crash_s in sorted(crash_set):
                for safe_s in sorted(safe_set):
                    examples.append({
                        "dt": dt,
                        "crash_solver": crash_s,
                        "crash_score": scores[crash_s],
                        "safe_solver": safe_s,
                        "safe_score": scores[safe_s],
                    })

    # 只保留每个 (crash_solver, safe_solver) 组合中 dt 最小的反例
    keyed = {}
    for ex in examples:
        k = (ex["crash_solver"], ex["safe_solver"])
        if k not in keyed or ex["dt"] < keyed[k]["dt"]:
            keyed[k] = ex

    path = os.path.join(OUTDIR, "counterexamples.csv")
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["dt","crash_solver","crash_score","safe_solver","safe_score"])
        writer.writeheader()
        for ex in sorted(keyed.values(), key=lambda x: x["dt"]):
            writer.writerow(ex)
    print(f"Counterexamples: {path} ({len(keyed)} found)")
    return keyed


def plot_heatmap(results):
    """生成反例热力图 PNG"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np

        dts = sorted(results.keys())
        solvers = SOLVERS
        data = np.zeros((len(solvers), len(dts)))
        for j, dt in enumerate(dts):
            for i, s in enumerate(solvers):
                data[i, j] = results[dt].get(s, 0.0)

        fig, ax = plt.subplots(figsize=(12, 5))
        im = ax.imshow(data, cmap="RdYlGn", aspect="auto", vmin=0, vmax=1)

        ax.set_xticks(range(len(dts)))
        ax.set_xticklabels([f"{dt:.3f}" for dt in dts], fontsize=9, rotation=45)
        ax.set_yticks(range(len(solvers)))
        ax.set_yticklabels([SOLVER_NAMES[s] for s in solvers], fontsize=11)
        ax.set_xlabel("Step Size (s)", fontsize=12)
        ax.set_ylabel("Solver", fontsize=12)
        ax.set_title("Safety Heatmap: Solver × Step Size\n(Green=Safe >0.8, Yellow=Warn, Red=Crash <=0.3)", fontsize=13)

        for i in range(len(solvers)):
            for j in range(len(dts)):
                val = data[i, j]
                color = "white" if val < 0.5 else "black"
                ax.text(j, i, f"{val:.2f}", ha="center", va="center", fontsize=8, color=color, fontweight="bold")

        fig.colorbar(im, ax=ax, label="Safety Score (overall)")
        fig.tight_layout()
        path = os.path.join(OUTDIR, "safety_heatmap.png")
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"Heatmap: {path}")
    except ImportError:
        print("(matplotlib not available, skip heatmap)")


def print_report(results, keyed):
    """打印人类可读的反例报告"""
    print("\n" + "=" * 75)
    print("COUNTEREXAMPLE MINING REPORT")
    print("=" * 75)
    print(f"Scenario: {NUM_DRONES} drones, obstacles={'ON' if OBSTACLES else 'OFF'}, {SIM_DURATION}s simulation")
    print()

    # 安全边界概览
    print("--- Per-Solver Critical Step Size (overall_score > 0.8) ---")
    for solver in SOLVERS:
        crit = LOW_DT
        for dt in sorted(results.keys()):
            if results[dt].get(solver, 0) > 0.8:
                crit = dt
            else:
                break
        print(f"  {SOLVER_NAMES[solver]:12s}: safe up to dt={crit:.4f}s")

    # 网格总览
    print("\n--- Safety Grid (S=Safe >0.8, W=Warn >0.3, X=Crash <=0.3) ---")
    header = f"{'dt':>8s} " + " ".join(f"{SOLVER_NAMES[s][:4]:>5s}" for s in SOLVERS)
    print(header)
    print("-" * len(header))
    for dt in sorted(results.keys()):
        row = f"{dt:8.4f} "
        for s in SOLVERS:
            sc = results[dt].get(s, 0)
            mark = "  S  " if sc > 0.8 else ("  W  " if sc > 0.3 else "  X  ")
            row += mark
        print(row)

    # 关键反例
    print("\n--- Key Counterexamples (minimal dt where safety diverges) ---")
    if not keyed:
        print("  No counterexamples found — try larger HIGH_DT or more stress")
    else:
        print(f"{'dt':>8s}  {'Crashed':12s} {'score':>6s}  {'Survived':12s} {'score':>6s}")
        print("-" * 60)
        for ex in sorted(keyed.values(), key=lambda x: x["dt"]):
            print(f"{ex['dt']:8.4f}  {SOLVER_NAMES[ex['crash_solver']]:12s} {ex['crash_score']:6.3f}  "
                  f"{SOLVER_NAMES[ex['safe_solver']]:12s} {ex['safe_score']:6.3f}")

    # 论文级总结
    print("\n--- Thesis-Level Summary ---")
    if keyed:
        for ex in sorted(keyed.values(), key=lambda x: x["dt"]):
            print(f"  At dt={ex['dt']:.3f}s: {SOLVER_NAMES[ex['crash_solver']]} crashes (score={ex['crash_score']:.2f}) "
                  f"while {SOLVER_NAMES[ex['safe_solver']]} flies safely (score={ex['safe_score']:.2f}). "
                  f"This demonstrates that solver choice directly determines flight safety "
                  f"at identical operating conditions.")


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    print("=" * 60)
    print("Counterexample Mining — Finding \"A crashes, B flies\" scenarios")
    print(f"  {len(SOLVERS)} solvers × {len(DT_GRID)} step sizes = {len(SOLVERS) * len(DT_GRID)} runs")
    print(f"  Drones: {NUM_DRONES}, Obstacles: {OBSTACLES}, Duration: {SIM_DURATION}s")
    print("=" * 60)
    print()

    results = mine_grid()
    write_grid_csv(results)
    plot_heatmap(results)
    keyed = find_counterexamples(results)
    print_report(results, keyed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
