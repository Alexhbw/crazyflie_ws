# 面向无人机安全自主飞行的 ODE 初值问题数值解法稳定域分析与安全边界形式化验证

基于 **Crazyflie 2.1** 微型四旋翼的 12-DOF 非线性 ODE 数值仿真、在线安全控制与形式化验证平台。

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B)](https://en.cppreference.com/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)](https://www.python.org/)
[![Eigen3](https://img.shields.io/badge/Eigen-3.x-blue)](https://eigen.tuxfamily.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## 📋 目录

- [项目概述](#项目概述)
- [系统架构](#系统架构)
- [环境依赖](#环境依赖)
- [编译与安装](#编译与安装)
- [快速开始](#快速开始)
- [运行模式](#运行模式)
  - [1. 正常仿真模式](#1-正常仿真模式)
  - [2. 安全监视模式](#2-安全监视模式)
  - [3. 单次数据采集](#3-单次数据采集)
  - [4. 快速求解器对比](#4-快速求解器对比)
  - [5. 多步长对比](#5-多步长对比)
  - [6. 全因子实验扫描](#6-全因子实验扫描)
  - [7. 临界步长二分搜索](#7-临界步长二分搜索)
  - [8. 反例自动挖掘](#8-反例自动挖掘)
  - [9. Monte Carlo 对比](#9-monte-carlo-对比)
- [ROS 参数参考](#ros-参数参考)
- [核心模块详解](#核心模块详解)
  - [五大 ODE 求解器](#五大-ode-求解器)
  - [安全监视器 SafetyMonitor](#安全监视器-safetymonitor)
  - [区间可达集 IntervalReachability](#区间可达集-intervalreachability)
  - [在线安全控制器 SafetyController](#在线安全控制器-safetycontroller)
  - [反例挖掘 Counterexample Mining](#反例挖掘-counterexample-mining)
  - [多步可达管 + Monte Carlo](#多步可达管--monte-carlo)
- [Python 工具脚本](#python-工具脚本)
- [文件结构](#文件结构)
- [关键实验发现](#关键实验发现)
- [Git 版本分支](#git-版本分支)
- [报告与文档](#报告与文档)
- [引用](#引用)

---

## 项目概述

本项目围绕 **"面向无人机安全自主飞行的 ODE 初值问题数值解法稳定域分析与安全边界形式化验证"** 这一核心课题，完成了从理论建模、数值分析、形式化验证、工程仿真、在线安全控制到实验自动化的**全链路系统性工作**。

### 核心贡献

| 维度 | 内容 |
|------|------|
| **数值分析** | 5 种 ODE 求解器（Euler, Heun, RK4, RK45, Implicit Euler）的 C++ 实现与稳定域对比 |
| **形式化验证** | 四维 □-不变式在线监控、区间可达集传播、反例自动挖掘 |
| **安全控制** | 三级递进式闭环安全控制器（EMA 平滑 + 自适应求解器切换） |
| **工程实现** | ROS 2 Humble 集群仿真平台，支持多机编队、人工势场避障、RViz 可视化 |
| **实验自动化** | 全因子参数扫描（150 次）、临界步长二分搜索、6 维评估指标体系 |

### 硬件平台

**Bitcraze Crazyflie 2.1** — 开源微型四旋翼（27g），物理参数全透明，转动惯量 10⁻⁵ 量级，数值敏感性极强，是验证 ODE 稳定域理论的理想平台。

| 参数 | 数值 |
|------|------|
| 质量 m | 0.027 kg |
| I_xx, I_yy | 1.395 × 10⁻⁵ kg·m² |
| I_zz | 2.235 × 10⁻⁵ kg·m² |
| 最大推力 | 0.6 N |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    run_experiment.sh                     │
│  (启动器: RSP × N + RViz2 + ode_engine_node)             │
└──────────────────────┬──────────────────────────────────┘
                       │
         ┌─────────────▼─────────────┐
         │  ode_engine_node.cpp      │
         │  (主仿真节点, ~760 行)     │
         │                          │
         │  ┌─────────────────────┐ │
         │  │ ODE Solvers (5 种)   │ │  ← solver_registry.hpp
         │  │ 统一 SolverFn 调度   │ │
         │  └────────┬────────────┘ │
         │           │               │
         │  ┌────────▼────────────┐ │
         │  │ Cascade PID (4 级)  │ │
         │  │ 位置→速度→姿态→角速率│ │
         │  └────────┬────────────┘ │
         │           │               │
         │  ┌────────▼────────────┐ │
         │  │ Artificial Potential │ │
         │  │ Field 避障 (t³ 势场) │ │  ← obstacles.yaml
         │  └────────┬────────────┘ │
         │           │               │
         │  ┌────────▼────────────┐ │
         │  │ SafetyMonitor       │ │  ← safety_monitor.hpp
         │  │ (4 维 □-不变式检查) │ │
         │  └────────┬────────────┘ │
         │           │               │
         │  ┌────────▼────────────┐ │
         │  │ SafetyController    │ │
         │  │ (三级递进保护)      │ │
         │  └────────┬────────────┘ │
         │           │               │
         │  ┌────────▼────────────┐ │
         │  │ IntervalReachability│ │  ← interval_reachability.hpp
         │  │ (区间可达集+可达管) │ │
         │  └─────────────────────┘ │
         └──────────────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
    ┌──────────┐ ┌──────────┐ ┌──────────┐
    │  RViz2   │ │  TF/odom │ │ CSV 输出 │
    │ 可视化   │ │ 状态发布 │ │ 指标/日志│
    └──────────┘ └──────────┘ └──────────┘

Python 实验层:
  run_experiments.py          → 全因子扫描 (150 次)
  safety_boundary_analysis.py → 临界步长二分搜索
  counterexample_mining.py    → 反例网格挖掘
  monte_carlo_compare.py      → MC vs 确定性区间
  plot_results.py             → 6 张报告级图表
```

---

## 环境依赖

| 组件 | 版本/说明 |
|------|-----------|
| **Ubuntu** | 22.04 LTS (Jammy) |
| **ROS 2** | Humble Hawksbill |
| **C++ 编译器** | GCC 11+ (C++17) |
| **Eigen3** | 3.4+ (线性代数) |
| **yaml-cpp** | 0.7+ (配置文件解析) |
| **Python** | 3.10+ |
| **pip 包** | `python-docx` `matplotlib` `numpy` `python-pptx` |

### 安装系统依赖

```bash
# ROS 2 Humble (Ubuntu 22.04)
# 参考: https://docs.ros.org/en/humble/Installation.html

# Eigen3 & yaml-cpp
sudo apt install libeigen3-dev libyaml-cpp-dev

# Python 依赖
pip install matplotlib numpy python-docx python-pptx
```

---

## 编译与安装

```bash
cd ~/crazyflie_ws

# 编译
colcon build --packages-select crazyflie_ode_core --symlink-install

# 加载环境
source install/setup.bash
```

> **注意**: 使用 `--symlink-install` 可以避免每次修改 Python 脚本后重新编译。

---

## 快速开始

最简单的启动方式——单机、RK4、步长 0.01s、有障碍物、RViz 可视化：

```bash
./run_experiment.sh rk4 0.01 1 true
```

参数说明: `<solver> <step_size> <num_drones> <enable_obstacles>`

---

## 运行模式

### 1. 正常仿真模式

启动 RViz 可视化 + 无人机集群仿真，适合日常开发和演示。

```bash
./run_experiment.sh rk4 0.01 4 true
```

- 自动启动 `robot_state_publisher`（每架无人机 × 1）
- 自动启动 `rviz2`（加载 `swarm.rviz` 配置）
- 支持 Ctrl+C 一键清理所有进程

### 2. 安全监视模式

每步检查四维安全条件，终端实时输出违规告警。

```bash
./run_safety_monitor.sh rk4 0.01 4
```

### 3. 单次数据采集

无 GUI 模式，运行指定时长后自动退出并写入指标 CSV。

```bash
./run_collect_data.sh rk4 0.01 4 60 true
```

### 4. 快速求解器对比

5 个求解器 × 固定步长 × 15s 仿真，~2 分钟完成。

```bash
./run_quick_compare.sh
```

输出: `compare_results/quick_compare.csv`

### 5. 多步长对比

5 求解器 × 3 步长，~5 分钟完成。

```bash
./run_multi_dt_compare.sh
```

### 6. 全因子实验扫描

150 次独立实验（5 求解器 × 5 步长 × 3 飞机数 × 2 障碍物），~25 分钟。

```bash
python3 src/crazyflie_ode_core/scripts/run_experiments.py
```

输出: `comparison_results/comparison_results.csv`

### 7. 临界步长二分搜索

对每个求解器二分搜索安全飞行的最大步长，~30 分钟。

```bash
python3 src/crazyflie_ode_core/scripts/safety_boundary_analysis.py
```

### 8. 反例自动挖掘

5 求解器 × 10 步长安全网格扫描，挖掘"A 崩溃 B 安全"的最小差异场景。

```bash
python3 src/crazyflie_ode_core/scripts/counterexample_mining.py
```

输出:
- `counterexample_results/solver_safety_grid.csv` — 完整网格数据
- `counterexample_results/counterexamples.csv` — 关键反例列表
- `counterexample_results/safety_heatmap.png` — 安全热力图

### 9. Monte Carlo 对比

200 次随机扰动仿真 vs 确定性区间分析。

```bash
python3 src/crazyflie_ode_core/scripts/monte_carlo_compare.py
```

输出:
- `monte_carlo_results/monte_carlo_distribution.png` — 分布直方图
- `monte_carlo_results/interval_vs_montecarlo.png` — 区间 vs MC 对比
- `monte_carlo_results/summary.json` — 统计数据

---

## ROS 参数参考

### 仿真参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `solver` | string | `rk4` | 求解器: `euler` / `heun` / `rk4` / `rk45` / `implicit` |
| `step_size` | double | `0.01` | 积分步长 (s) |
| `num_drones` | int | `1` | 无人机数量 |
| `enable_obstacles` | bool | `true` | 障碍物避障开关 |

### 安全监视器参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `safety.v_max` | double | `2.5` | 速度安全阈值 (m/s) |
| `safety.theta_max` | double | `0.8` | 姿态安全阈值 (rad) |
| `safety.energy_drift_eps` | double | `0.01` | 能量漂移阈值 (J/步) |
| `safety.warmup_time` | double | `2.0` | 预热时间 (s)，期间跳过碰撞/姿态检查 |
| `safety.log_path` | string | `safety_log.csv` | 违规日志路径 |
| `safety.verbose` | bool | `false` | 实时打印违规到终端 |

### 实验模式参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `experiment.enabled` | bool | `false` | 启用实验模式 |
| `experiment.duration` | double | `30.0` | 仿真时长 (s)，到期自动退出 |
| `experiment.metrics_path` | string | `experiment_metrics.csv` | 指标输出路径 |
| `experiment.headless` | bool | `false` | 跳过所有 ROS 发布（纯计算，加速 3-5×） |

### 区间可达集参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `reachability.enabled` | bool | `false` | 启用区间可达集分析 |
| `reachability.epsilon` | double | `0.01` | 状态扰动幅度 |
| `reachability.method` | string | `sampling` | 方法: `sampling` (25次/步) / `jacobian` (13次/步) |

### 安全控制器参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `safety_controller.enabled` | bool | `false` | 启用在线安全控制器 |
| `safety_controller.threshold_halve` | double | `0.7` | L1: 减半步长的分数阈值 |
| `safety_controller.threshold_switch` | double | `0.3` | L2: 切换求解器的分数阈值 |
| `safety_controller.threshold_stop` | double | `0.05` | L3: 紧急关机的分数阈值 |

---

## 核心模块详解

### 五大 ODE 求解器

所有求解器通过 `SolverFn` 统一函数签名实现零分支开销调度：

```cpp
using SolverFn = std::function<Vector12d(const Vector12d&, const Vector4d&, double)>;
```

| 求解器 | 阶数 | 单步 f(x) 求值 | 稳定性 | 安全临界步长 |
|--------|------|---------------|--------|-------------|
| **Euler** (前向欧拉) | O(h) | 1 次 | 条件稳定 | ≤ 0.02 s |
| **Heun** (改进欧拉/RK2) | O(h²) | 2 次 | 条件稳定 | ≤ 0.02 s |
| **RK4** (经典 Runge-Kutta) | O(h⁴) | 4 次 | 条件稳定 | ≤ 0.02 s |
| **RK45** (Runge-Kutta-Fehlberg) | O(h⁴)/O(h⁵) | 6 + 自适应子步 | 自适应 | **≤ 0.8 s** ⭐ |
| **Implicit Euler** (隐式欧拉) | O(h) | ~65 次 (Newton 迭代) | **A-稳定** | ≤ 0.02 s |

> **关键发现**: RK45 的自适应"拒绝-重试"机制使其在 0.05~0.8 s 大步长下成为唯一安全飞行的求解器。

### 安全监视器 SafetyMonitor

在每一时间步自动检查四个形式化安全条件（□-不变式）：

| 条件 | 不变式 | 阈值 | 类型 |
|------|--------|------|------|
| **C1 碰撞安全** | □(min d_surf > 0) | 0 m | 硬安全 |
| **C2 速度安全** | □(\|v_i\| < v_max) | 2.5 m/s | 硬安全 |
| **C3 姿态安全** | □(\|θ_i\| < θ_max) | 0.8 rad | 硬安全 |
| **C4 能量漂移** | □(\|ΔE\| < ε) | 0.01 J/步 | 软诊断 |

安全分数: `overall_score = min(collision, velocity, attitude)`（木桶原理）

### 区间可达集 IntervalReachability

给定当前状态 x 和有界扰动 ε，计算一步积分后的状态区间包围盒，保守检测是否与障碍物相交。

| 方法 | 求解器调用/步 | 保守性 | 适用场景 |
|------|-------------|--------|----------|
| **采样法** | 25 次 (1中心 + 12维×2方向) | 强 | 离线验证 |
| **Jacobian 法** | 13 次 (1中心 + 12维) | 中 | 在线预警 |

多步扩展: `computeReachTube(k, ...)` 迭代 k 步，提供安全时间窗口预测。

### 在线安全控制器 SafetyController

将安全监视与求解器调度闭环，EMA 平滑后按三级递进响应：

```
smoothed_score  <  0.7   →  L1: REDUCE_DT      (步长减半)
                <  0.3   →  L2: SWITCH_SOLVER   (切换更保守求解器)
                <  0.05  →  L3: EMERGENCY_STOP  (紧急关机)
```

求解器安全层级（不可逆，单调递增）: `Euler → Heun → RK4 → RK45 → Implicit Euler`

### 反例挖掘 Counterexample Mining

5 求解器 × 10 步长 (0.01~2.0 s) 安全网格扫描，挖掘"同一 dt 下 A 崩溃 B 安全"的反例。

**3 个关键反例 (dt=0.05 s)**:

| 崩溃求解器 | 分数 | 安全求解器 | 分数 |
|-----------|------|-----------|------|
| Heun | 0.07 | RK45 | 1.00 |
| RK4 | 0.05 | RK45 | 1.00 |
| Implicit Euler | 0.03 | RK45 | 1.00 |

### 多步可达管 + Monte Carlo

- **可达管**: 从单步区间传播扩展至 k 步，形成"名义轨迹 + 误差包络管"
- **Monte Carlo**: 200 次随机扰动仿真 vs 确定性区间，验证覆盖度

| 指标 | 确定性区间 | Monte Carlo | 结论 |
|------|-----------|-------------|------|
| 覆盖率 | 100% | — | 区间法不遗漏任何可能结果 |
| 过近似度 | ~4× (vs σ) | — | 保守但工程可用 |

---

## Python 工具脚本

所有脚本位于 `src/crazyflie_ode_core/scripts/`：

| 脚本 | 功能 | 耗时 |
|------|------|------|
| `run_experiments.py` | 全因子参数扫描 (150 次) | ~25 min |
| `safety_boundary_analysis.py` | 临界步长二分搜索 | ~30 min |
| `counterexample_mining.py` | 反例网格挖掘 + 热力图 | ~8 min |
| `monte_carlo_compare.py` | Monte Carlo vs 确定性区间 | ~20 min |
| `plot_results.py` | 6 张报告级图表生成 | < 1 s |

顶层辅助脚本：

| 脚本 | 功能 |
|------|------|
| `build_v3_report.py` | 生成完整 DOCX 报告 (14 章) |
| `build_ppt_v3.py` | 生成 V3 新增内容 PPT (10 张幻灯片) |

---

## 文件结构

```
crazyflie_ws/
├── run_experiment.sh               # 正常仿真启动
├── run_safety_monitor.sh           # 安全监视模式
├── run_quick_compare.sh            # 快速求解器对比
├── run_multi_dt_compare.sh         # 多步长对比
├── run_collect_data.sh             # 单次数据采集
├── build_v3_report.py              # DOCX 报告生成
├── build_ppt_v3.py                 # PPT 生成
├── poster_prompt_v3_final.md       # 海报生成 Prompt
│
├── src/crazyflie_ode_core/
│   ├── CMakeLists.txt              # 构建配置
│   ├── package.xml                 # 包清单
│   │
│   ├── include/crazyflie_ode_core/
│   │   ├── solver_registry.hpp     # 求解器枚举 + SolverFn 定义
│   │   ├── safety_monitor.hpp      # SafetyMonitor + SafetyController
│   │   └── interval_reachability.hpp # IntervalReachability + TubeStep
│   │
│   ├── src/
│   │   ├── ode_engine_node.cpp     # 主仿真节点 (~760 行)
│   │   ├── safety_monitor.cpp      # 安全监视 + 安全控制器
│   │   └── interval_reachability.cpp # 区间传播 + AABB碰撞检测
│   │
│   ├── scripts/
│   │   ├── run_experiments.py      # 全因子扫描
│   │   ├── safety_boundary_analysis.py # 临界步长搜索
│   │   ├── counterexample_mining.py    # 反例挖掘
│   │   ├── monte_carlo_compare.py      # MC 对比
│   │   └── plot_results.py             # 图表生成
│   │
│   ├── config/
│   │   └── obstacles.yaml          # 5 根圆柱障碍物定义
│   │
│   ├── launch/
│   │   └── sim_launch.py           # ROS 2 Launch 文件
│   │
│   ├── rviz/
│   │   ├── sim.rviz                # 单机 RViz 配置
│   │   └── swarm.rviz              # 集群 RViz 配置 (Transient Local QoS)
│   │
│   ├── urdf/
│   │   └── crazyflie.urdf          # 无人机 URDF 模型
│   │
│   └── meshes/
│       └── crazyflie2.dae          # 3D 网格文件
│
├── counterexample_results/         # 反例挖掘输出
│   ├── safety_heatmap.png
│   └── counterexamples.csv
│
├── monte_carlo_results/            # MC 对比输出
│   ├── monte_carlo_distribution.png
│   ├── interval_vs_montecarlo.png
│   └── summary.json
│
└── compare_results/                # 求解器对比输出
    └── quick_compare.csv
```

---

## 关键实验发现

### 发现一: 安全步长 < 理论稳定步长

安全边界比数值稳定边界窄 **3~5 倍**。数值稳定只是安全飞行的必要条件，而非充分条件。安全还需满足物理约束（障碍物距离、速度上限、姿态限制）。

### 发现二: RK45 自适应法独树一帜

在 0.05~0.8 s 大步长范围内，RK45 是**唯一**能安全飞行的求解器。其"拒绝-重试"机制——当局部截断误差超过容差时作废当前步并子步重算——确保每个输出帧的精度始终在允许范围内。

### 发现三: A-稳定 ≠ 安全飞行

Implicit Euler 具有 A-稳定性（永不数值发散），但在 0.05 s 步长下轨迹精度严重退化，导致无人机因跟踪误差过大而碰撞障碍物。这一反例直接证明了"数值稳定 ≠ 安全飞行"。

### 发现四: 确定性区间 100% 覆盖 Monte Carlo 样本

区间可达集方法在 200 次随机扰动测试中实现了 100% 覆盖率，过近似度约 4×。用适度的保守代价换取绝对安全保证，是安全关键系统的标准工程实践。

---

## Git 版本分支

| 分支/标签 | 内容 |
|-----------|------|
| `master` (v4) | 完整系统: SafetyController + Counterexample Mining + Reach Tube + Monte Carlo |
| `v3-safety-controller` | 第三版: 在线安全控制器 + 实验 + 安全监视 + 障碍物 |
| `v2-experiments` | 第二版: SafetyMonitor + IntervalReachability + Experiment Framework + Obstacle System |
| `v1-original` | 第一版: 原始 ODE 求解器 + ROS 2 仿真框架 |

---

## 报告与文档

| 文件 | 说明 |
|------|------|
| `v3完整报告_面向无人机安全自主飞行的ODE初值问题数值解法稳定域分析与安全边界形式化验证.docx` | 最终版完整报告 (14 章) |
| `安全控制器与反例挖掘_V3新增_PPT.pptx` | V3 新增内容 PPT (10 张) |
| `poster_prompt_v3_final.md` | 海报生成 Prompt (16:9) |

---

## 引用

如果您使用了本项目或其中的方法，请引用：

```bibtex
@mastersthesis{crazyflie-ode-safety-2026,
  title     = {面向无人机安全自主飞行的ODE初值问题数值解法稳定域分析与安全边界形式化验证},
  author    = {Alex},
  year      = {2026},
  school    = {},
  note      = {基于 Crazyflie 2.1 的数值仿真、在线安全控制与形式化验证平台},
  url       = {https://github.com/your-org/crazyflie-ws}
}
```

---

## 许可证

MIT License

---

*2026 年 6 月*
