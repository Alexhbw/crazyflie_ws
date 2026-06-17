# 障碍物避障模块 — 设计与实现报告

## 1. 概述

在原有的 Crazyflie 集群 ODE 数值仿真系统中，新增了**圆柱体障碍物避障子系统**。该模块支持：

- 从 YAML 配置文件加载任意数量圆柱障碍物
- 基于平滑人工势场 + 速度阻尼 + 切向引导的三层避障算法
- ROS 2 参数动态开关（无需重新编译）
- RViz 实时可视化（半透明红色圆柱 + Transient Local QoS）

---

## 2. 系统架构

```
obstacles.yaml (配置)  ──→  loadObstaclesFromYAML()  ──→  std::vector<CylinderObstacle>
                                   │
                    ┌──────────────┴──────────────┐
                    ▼                              ▼
     computeObstacleRepulsion(pos, vel)    publishObstacleMarkers()
     人工势场排斥加速度 (3D vector)          RViz Marker 可视化
                    │                              │
                    ▼                              ▼
        cascade_pid_control()               /obstacles topic
        acc_des += a_rep                  (Transient Local QoS)
```

---

## 3. 数据结构与配置文件

### 3.1 数据结构 (`ode_engine_node.cpp` 第 26–31 行)

```cpp
struct CylinderObstacle {
    std::string id;       // 唯一标识符
    double x, y, z;       // 圆柱体中心位置 (世界坐标)
    double radius;        // 圆柱半径
    double height;        // 圆柱高度
};
```

### 3.2 YAML 配置文件 (`config/obstacles.yaml`)

```yaml
obstacles:
  - id: "center_pole"
    x: 0.0; y: 0.0; z: 0.75
    radius: 0.05; height: 1.5   # 中心细杆 (r=0.05, 呼吸圆环收缩时可穿过)

  - id: "east_pillar"
    x: 1.6; y: 0.0; z: 1.0
    radius: 0.18; height: 2.0   # 东侧粗柱 (位于编队公转轨迹上)

  - id: "west_pillar"
    x: -1.6; y: 0.0; z: 1.0    # 西侧对称柱
    radius: 0.18; height: 2.0

  - id: "north_pillar"
    x: 0.0; y: 1.6; z: 0.8
    radius: 0.12; height: 1.6   # 北侧柱

  - id: "south_pillar"
    x: 0.0; y: -1.6; z: 0.8
    radius: 0.12; height: 1.6   # 南侧柱
```

布局逻辑：中心细杆位于呼吸圆环中心 (0, 0)，四个方向各有一根粗柱位于编队公转半径 ~1.5 m 附近，形成 **"中心标定杆 + 四角障碍柱"** 的障碍场。

### 3.3 ROS 参数控制

新增 ROS 2 参数 `enable_obstacles`（布尔型，默认 `true`），在第 56–58 行声明：

```cpp
this->declare_parameter<bool>("enable_obstacles", true);
this->get_parameter("enable_obstacles", enable_obstacles_);
if (enable_obstacles_) { loadObstaclesFromYAML(); }
```

使用方式：

```bash
# 开启障碍物
./run_experiment.sh rk4 0.01 5

# 关闭障碍物（第 4 参数设为 false）
./run_experiment.sh rk4 0.01 5 false

# 或直接 ros2 run
ros2 run crazyflie_ode_core ode_engine_node --ros-args \
  -p solver:=rk4 -p step_size:=0.01 -p num_drones:=5 \
  -p enable_obstacles:=false
```

---

## 4. 避障算法：平滑势场 + 速度阻尼 + 切向引导

### 4.1 算法设计目标

| 需求 | 解法 |
|---|---|
| 远距离不干扰编队 | 仅在 `d_surf < d_safe` 时生效 |
| 靠近时平滑减速 | 速度阻尼项：接近越快、制动越强 |
| 表面附近不弹飞 | 三次方 `t³` 力场曲线，边界平滑过渡 |
| 顺势绕行而非硬撞 | 切向引导力，沿速度方向偏转 |
| 穿透后紧急弹出 | `d_surf ≤ 0` 时触发 3× 满力径向推出 |

### 4.2 无量纲侵入比

对每个障碍物计算无人机到障碍物表面的最近距离 \(d_{\text{surf}}\)：

\[
d_{\text{surf}} = d_{xy} - r_{\text{obs}}
\]

定义侵入比 \(t \in [0, 1]\)：

\[
t = \text{clamp}\!\left(1 - \frac{d_{\text{surf}}}{d_{\text{safe}}},\; 0,\; 1\right)
\]

- \(t = 0\)：在安全边界外，无斥力
- \(t = 1\)：在障碍物表面上，斥力最大

### 4.3 力场四分量

完整斥力加速度由四个分量叠加（第 160–241 行）：

#### (a) 径向排斥

\[
\mathbf{F}_{\text{rad}} = k_{\text{rep}} \cdot t^{3} \cdot \hat{\mathbf{r}}_{xy}
\]

其中 \(\hat{\mathbf{r}}_{xy}\) 为无人机指向障碍物轴心的单位径向向量。

- 采用 **三次方** \(t^3\) 而非 \(t^2\)：在安全边界（低 \(t\)）更加柔和，在表面附近（高 \(t\)）急剧增强
- 穿透应急：当 \(d_{\text{surf}} \leq 0\)（已穿透表面），施以 \(3k_{\text{rep}}\) 满力径向推出

设计迭代历程：

| 版本 | 力场公式 | 问题 |
|---|---|---|
| v1 | \(F = k \cdot (1/d_{\text{surf}} - 1/d_{\text{safe}}) / d_{\text{surf}}^2\)（经典 1/r 势场） | 靠近表面时力趋向无穷，无人机被猛烈弹飞 |
| v2 | \(F = k \cdot t^2\)（二次平滑） | 远场效果好，但 \(k=2.0\) 时表面力仅 2 m/s²，无法抵抗编队控制器 (4 m/s²)，无人机直接穿透 |
| **v3（最终）** | \(F = k \cdot t^3\)（三次平滑），\(k=6.0\) | 远场柔和 (t=0.5 时仅 0.75 m/s²)，表面强劲 (t=1 时 6.0 m/s²)，完美平衡 |

#### (b) 速度阻尼

\[
\mathbf{F}_{\text{damp}} = -k_{\text{rep}} \cdot 0.6 \cdot t^2 \cdot v_{\text{radial}} \cdot \hat{\mathbf{r}}_{xy} \quad (\text{if } v_{\text{radial}} > 0)
\]

其中 \(v_{\text{radial}} = \mathbf{v}_{xy} \cdot \hat{\mathbf{r}}_{xy}\) 为朝向障碍物轴心的径向速度分量。

- **物理意义**：这是"刹车"，而非"推力"。无人机冲向障碍物越快，制动力越强
- **防止穿透的关键机制**：即使 PID 控制指令朝向障碍物，该阻尼项会主动减速，使无人机在到达表面之前就停下来
- 仅当 \(v_{\text{radial}} > 0\)（朝向障碍物）时生效，离开时不施加额外制动力

#### (c) 切向绕行引导

\[
\mathbf{F}_{\text{tan}} = k_{\text{rep}} \cdot 0.35 \cdot t_{\text{tan}}^3 \cdot \hat{\mathbf{t}}
\]

其中 \(\hat{\mathbf{t}}\) 为径向方向逆时针旋转 90° 得到的切向方向：

\[
\hat{\mathbf{t}} = (-\hat{r}_y,\; \hat{r}_x,\; 0)
\]

- 启动条件：\(d_{\text{surf}} < 0.7 \cdot d_{\text{safe}}\)（更靠近表面才触发）
- 切向符号选择：取与当前速度分量方向一致的一侧，实现 **"顺势绕行"**
- 物理意义：引导无人机从障碍物侧面滑过，而非正面对撞

#### (d) Z 轴纵向排斥

\[
\mathbf{F}_z = k_{\text{rep}} \cdot 0.5 \cdot t_z^3 \cdot \text{sign}(z - z_{\text{obs}}) \cdot \hat{\mathbf{z}}
\]

力度减半（系数 0.5），因为 Z 轴避障优先级低于 XY 平面避障。

### 4.4 合力上限

```cpp
double mag = a_rep.norm();
if (mag > 6.0) a_rep = a_rep * (6.0 / mag);  // 上限 6 m/s²
```

上限 6.0 m/s² **必须大于编队控制器的加速度上限 (4.0 m/s²)**，确保在障碍物表面附近斥力能覆盖编队指令。实际运行中，由于三次方曲线的特性，在安全边界处合力极小 (< 1 m/s²)，不会干扰正常编队。

### 4.5 与级联 PID 的集成

排斥加速度直接叠加到 PID 控制器的期望加速度上（第 325 行）：

```cpp
Vector3d acc_des = Kp_v * (vel_des - vel) + a_rep;  // a_rep 包含机间排斥 + 障碍物排斥
acc_des = acc_des.cwiseMin(4.0).cwiseMax(-4.0);     // 编队控制器限幅
```

总排斥力 = **机间排斥**（`compute_repulsive_acc`，1/r² 势场）+ **障碍物排斥**（`computeObstacleRepulsion`，平滑三次方势场）。

### 4.6 参数表

| 参数 | 符号 | 数值 | 说明 |
|---|---|---|---|
| 安全距离 | \(d_{\text{safe}}\) | 0.3 m | 障碍物排斥影响半径 |
| 排斥增益 | \(k_{\text{rep}}\) | 6.0 | 径向排斥峰值 (t=1 时) |
| 速度阻尼系数 | \(\alpha_{\text{damp}}\) | 0.6 \(k_{\text{rep}}\) | 朝向障碍物速度的制动增益 |
| 切向引导系数 | \(\alpha_{\text{tan}}\) | 0.35 \(k_{\text{rep}}\) | 切向绕行力的比例 |
| Z 轴系数 | \(\alpha_z\) | 0.5 \(k_{\text{rep}}\) | 纵向排斥减半 |
| 力场上限 | \(F_{\text{max}}\) | 6.0 m/s² | 总排斥力矢量模上限 |
| 切向触发比 | \(\beta_{\text{tan}}\) | 0.7 \(d_{\text{safe}}\) | 切向引导的触发距离 |

---

## 5. RViz 可视化

### 5.1 Marker 发布

使用 `visualization_msgs::msg::Marker::CYLINDER` 类型以半透明红色圆柱显示障碍物。

**关键设计**：

- **Transient Local QoS**（`rclcpp::QoS(20).transient_local()`）：DDS 中间件保留最近 20 条消息，RViz 后连也能立即收到全部标记
- **双函数分离**：
  - `publishObstacleMarkersInit()`：构造时调用一次，先 `DELETEALL` 清旧标记再 `ADD`
  - `publishObstacleMarkers()`：每 100 仿真步（约 1 秒）周期调用，仅发 `ADD`，**避免 DELETEALL 导致闪烁**

### 5.2 RViz 配置文件

在 `swarm.rviz` 和 `sim.rviz` 的 Display 列表中新增长 Marker 面板：

```yaml
- Class: rviz_default_plugins/Marker
  Enabled: true
  Name: Obstacles
  Topic:
    Durability Policy: Transient Local
    Value: /obstacles
```

### 5.3 视觉效果参数

```cpp
m.color.r = 1.0f;   m.color.g = 0.2f;   m.color.b = 0.2f;   // 红色
m.color.a = 0.6f;                                                    // 半透明
m.lifetime = rclcpp::Duration::from_seconds(0);                      // 永久显示
```

---

## 6. 调试过程中的关键问题与解决方案

| # | 问题 | 根因 | 解决 |
|---|---|---|---|
| 1 | RViz 不弹窗 | NVIDIA 驱动 570/580 版本冲突，GLX 创建 OpenGL 上下文失败 | 重启机器加载正确内核模块；临时方案 `LIBGL_ALWAYS_SOFTWARE=1` |
| 2 | 障碍物标记不显示 | 发布者使用普通 QoS，RViz 后连收不到消息 | 改用 `transient_local()` QoS |
| 3 | 标记闪烁（每秒消失重出现）| `publishObstacleMarkers()` 每次先发 `DELETEALL` | 拆分为初始初始化（含 DELETEALL）和周期重发（仅 ADD）两个函数 |
| 4 | 编队运动被完全打乱 | 中心柱位于呼吸圆环中心 (0,0)，当圆环收缩至 r=0.4m 时排斥力高达 160 m/s² | 中心柱半径 0.15→0.05m；降低 k_rep 和 d_safe；降低加速度上限 |
| 5 | 无人机直接穿透障碍物 | 力场曲线 t² 在表面附近力度不足 (2 m/s² < PID 的 4 m/s²) | 改用 t³ 曲线 + 提高 k_rep 至 6.0 + 新增速度阻尼 + 穿透应急 3× 推出 |

---

## 7. 文件改动清单

| 文件 | 状态 | 关键改动 |
|---|---|---|
| `src/ode_engine_node.cpp` | 新增约 140 行 | `CylinderObstacle` 结构体、(第 26 行) `loadObstaclesFromYAML()`、(第 124 行) `computeObstacleRepulsion()`、(第 160 行) `publishObstacleMarkersInit()`、(第 244 行) `publishObstacleMarkers()`、(第 254 行) `enable_obstacles` 参数、(第 56 行) Transient Local QoS 发布者、(第 53 行) 主循环中斥力计算集成、(第 452 行) 周期标记重发 |
| `config/obstacles.yaml` | 新建 | 5 个圆柱障碍物定义 (第 472 行) |
| `CMakeLists.txt` | 修改 | 新增 `yaml-cpp` 依赖 |
| `package.xml` | 修改 | 新增 `yaml-cpp` 依赖 |
| `rviz/swarm.rviz` | 修改 | 新增 Marker Display 面板，订阅 `/obstacles` |
| `rviz/sim.rviz` | 修改 | 同上 |
| `run_experiment.sh` | 修改 | 新增第 4 参数 `ENABLE_OBS`，传递给 `enable_obstacles` ROS 参数 |
