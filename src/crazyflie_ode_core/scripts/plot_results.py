#!/usr/bin/env python3
"""报告级图表生成器 — 自适应单/多步长数据"""

import csv, os, sys

try:
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

DEFAULT_CSV = "experiment_results/comparison_results.csv"
OUTDIR = "plots"

SOLVER_LABELS = {"euler":"Euler","heun":"Heun","rk4":"RK4","rk45":"RK45","implicit":"Implicit"}
SOLVER_COLORS = {"euler":"#e41a1c","heun":"#377eb8","rk4":"#4daf4a","rk45":"#984ea3","implicit":"#ff7f00"}
SOLVER_MARKERS = {"euler":"s","heun":"o","rk4":"D","rk45":"^","implicit":"v"}
SOLVER_ORDER = ["euler","heun","rk4","rk45","implicit"]


def load_data(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            r = {k.strip(): v.strip() for k, v in row.items()}
            if "sweep_solver" not in r:
                # quick_compare 格式: solver, energy_drift_rate, ...
                r["sweep_solver"] = r.get("solver","")
                r["sweep_step_size"] = r.get("step_size","0.01")
                r["sweep_num_drones"] = r.get("num_drones","4")
                obs = r.get("obstacles_enabled","true")
                r["sweep_obstacles"] = "True" if obs in ("true","True","1") else "False"
            # 确保所有关键字段非空
            if not r.get("sweep_num_drones"): r["sweep_num_drones"] = "4"
            if not r.get("sweep_obstacles"): r["sweep_obstacles"] = "True"
            if not r.get("sweep_step_size"): r["sweep_step_size"] = "0.01"
            rows.append(r)
    return rows


def pf(row, key, d=float("nan")):
    try: return float(row.get(key, d))
    except: return d


def has_multi_dt(rows):
    return len(set(r.get("sweep_step_size","") for r in rows)) > 1


def collect(rows, field, n_drones="4", obs="True"):
    data = {}
    for s in SOLVER_ORDER:
        pts = []
        for r in rows:
            if r.get("sweep_solver","") != s: continue
            if n_drones and r.get("sweep_num_drones","") != n_drones: continue
            if obs and r.get("sweep_obstacles","") != obs: continue
            dt = pf(r, "sweep_step_size")
            val = pf(r, field)
            if not np.isnan(dt) and not np.isnan(val):
                pts.append((dt, val))
        pts.sort()
        if pts: data[s] = pts
    return data


def bar_or_line(data, ax, multi):
    if multi:
        for s, pts in data.items():
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=SOLVER_COLORS[s], marker=SOLVER_MARKERS[s],
                    label=SOLVER_LABELS[s], linewidth=2, markersize=8)
        ax.set_xscale("log")
    else:
        sv = [(s, data[s][0][1]) for s in SOLVER_ORDER if s in data]
        if not sv: return
        ss, vs = zip(*sv)
        x = np.arange(len(ss))
        vmin = min(vs)
        vmax = max(vs)
        vrange = vmax - vmin if vmax > vmin else 1.0
        # 从最小值截断 y 轴, 放大差异
        y_bottom = max(0, vmin - vrange * 0.3)
        bars = ax.bar(x, vs, color=[SOLVER_COLORS[s] for s in ss], edgecolor="white")
        ax.set_ylim(bottom=y_bottom)
        for b, v in zip(bars, vs):
            ax.text(b.get_x()+b.get_width()/2, b.get_height(),
                    f"{v:.5f}", ha="center", va="bottom", fontsize=8, rotation=90)
        ax.set_xticks(x)
        ax.set_xticklabels([SOLVER_LABELS[s] for s in ss], fontsize=11)
        # 截断标注
        if y_bottom > 0:
            ax.text(0.5, y_bottom * 1.05, f"y-axis truncated at {y_bottom:.4f} to show differences",
                    transform=ax.get_xaxis_transform(), fontsize=8, color="grey", ha="center")


def fig1(rows, od):
    data = collect(rows, "energy_drift_rate")
    multi = has_multi_dt(rows)
    fig, ax = plt.subplots(figsize=(8,5))
    bar_or_line(data, ax, multi)
    ax.set_ylabel("Energy Drift Rate (J/s)", fontsize=12)
    ax.set_title("Stability: "+("Energy Drift vs Step Size" if multi else "Energy Drift Rate (dt=0.01, 4 drones)"), fontsize=13)
    if multi: ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{od}/01_energy_drift.png", dpi=150); plt.close()
    print(f"  -> {od}/01_energy_drift.png")


def fig2(rows, od):
    data = collect(rows, "min_obstacle_dist")
    multi = has_multi_dt(rows)
    fig, ax = plt.subplots(figsize=(8,5))
    bar_or_line(data, ax, multi)
    if multi: ax.axhline(y=0, color="red", linestyle="--", label="Collision")
    ax.set_ylabel("Min Obstacle Distance (m)", fontsize=12)
    ax.set_title("Safety: "+("Obstacle Clearance vs Step Size" if multi else "Obstacle Clearance (dt=0.01, 4 drones)"), fontsize=13)
    if multi: ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{od}/02_safety_distance.png", dpi=150); plt.close()
    print(f"  -> {od}/02_safety_distance.png")


def fig3(rows, od):
    data = collect(rows, "rms_position_error", obs="")
    multi = has_multi_dt(rows)
    fig, ax = plt.subplots(figsize=(8,5))
    bar_or_line(data, ax, multi)
    ax.set_ylabel("RMS Position Error (m)", fontsize=12)
    ax.set_title("Accuracy: "+("Tracking Error vs Step Size" if multi else "RMS Position Error (dt=0.01, 4 drones)"), fontsize=13)
    if multi: ax.legend(fontsize=10); ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{od}/03_accuracy.png", dpi=150); plt.close()
    print(f"  -> {od}/03_accuracy.png")


def fig4(rows, od):
    data = collect(rows, "avg_compute_ms", obs="")
    multi = has_multi_dt(rows)
    fig, ax = plt.subplots(figsize=(8,5))
    bar_or_line(data, ax, multi)
    ax.set_ylabel("Avg Compute Time (ms/step)", fontsize=12)
    ax.set_title("Performance: "+("Computation Cost vs Step Size" if multi else "Compute Time per Step (dt=0.01, 4 drones)"), fontsize=13)
    if multi: ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{od}/04_performance.png", dpi=150); plt.close()
    print(f"  -> {od}/04_performance.png")


def fig5(boundary_csv, od):
    if not os.path.exists(boundary_csv):
        print(f"  (skip: {boundary_csv} not found)")
        return
    rows = []
    with open(boundary_csv) as f:
        for r in csv.DictReader(f): rows.append(r)
    if not rows: return
    solvers = [r["solver"] for r in rows]
    safe = [float(r["safe_step_size"]) for r in rows]
    unsafe = [float(r["unsafe_step_size"]) for r in rows]
    margin = [float(r["margin"]) for r in rows]
    fig, ax = plt.subplots(figsize=(7,4))
    x = np.arange(len(solvers))
    ax.bar(x, safe, color="#4daf4a", label="Safe Step Size")
    ax.bar(x, margin, bottom=safe, color="#ff7f00", label="Safety Margin")
    for i,(s,u) in enumerate(zip(safe,unsafe)):
        ax.text(i, s+margin[i]/2, f"{u:.4f}", ha="center", va="center", fontsize=8, color="white", fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels([SOLVER_LABELS.get(s,s) for s in solvers], fontsize=11)
    ax.set_ylabel("Step Size (s)", fontsize=12)
    ax.set_title("Safety Boundary: Critical Step Size per Solver", fontsize=13)
    ax.legend(fontsize=10)
    fig.tight_layout(); fig.savefig(f"{od}/05_safety_boundary.png", dpi=150); plt.close()
    print(f"  -> {od}/05_safety_boundary.png")


def fig6(rows, od):
    cats = ["Stability", "Safety", "Speed Ctrl", "Accuracy", "Real-time"]
    N = len(cats)
    angles = np.linspace(0, 2*np.pi, N, endpoint=False).tolist()
    angles += angles[:1]
    fig, ax = plt.subplots(figsize=(7,7), subplot_kw=dict(polar=True))
    for s in SOLVER_ORDER:
        score = []
        for r in rows:
            if r.get("sweep_solver","")==s and r.get("sweep_num_drones","")=="4" and r.get("sweep_obstacles","")=="True":
                ed = pf(r,"energy_drift_rate",0)
                md = pf(r,"min_obstacle_dist",-1)
                vm = pf(r,"max_velocity",10)
                pe = pf(r,"rms_position_error",10)
                cp = pf(r,"avg_compute_ms",100)
                score = [
                    max(0, 1-ed/0.002),
                    max(0, min(1, (md+0.2)/0.7)),
                    max(0, 1-vm/5.0),
                    max(0, 1-pe/3.0),
                    max(0, 1-cp/1.0),
                ]
                break
        if score:
            vals = score + score[:1]
            ax.fill(angles, vals, alpha=0.1, color=SOLVER_COLORS[s])
            ax.plot(angles, vals, color=SOLVER_COLORS[s], linewidth=2, label=SOLVER_LABELS[s], marker="o")
    ax.set_xticks(angles[:-1]); ax.set_xticklabels(cats, fontsize=10)
    ax.set_ylim(0,1); ax.set_title("Solver Radar (dt=0.01, 4 drones)", fontsize=13, pad=25)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3,1.1), fontsize=10)
    fig.tight_layout(); fig.savefig(f"{od}/06_radar.png", dpi=150); plt.close()
    print(f"  -> {od}/06_radar.png")


def main():
    if not HAS_PLOT:
        print("Need: pip3 install matplotlib numpy"); return 1
    csv_path = sys.argv[1] if len(sys.argv)>1 else DEFAULT_CSV
    od = sys.argv[3] if len(sys.argv)>3 else OUTDIR
    if not os.path.exists(csv_path):
        print(f"Not found: {csv_path}"); return 1
    os.makedirs(od, exist_ok=True)
    rows = load_data(csv_path)
    print(f"Loaded {len(rows)} rows from {csv_path}")

    fig1(rows, od)
    fig2(rows, od)
    fig3(rows, od)
    fig4(rows, od)
    fig5("safety_boundary_results/safety_boundary_table.csv", od)
    fig6(rows, od)
    print(f"\nDone. Charts in {od}/")
    return 0

if __name__=="__main__":
    sys.exit(main())
