import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rosbags.highlevel import AnyReader
from pathlib import Path

# --- 新增核心导入：引入类型库系统 ---
from rosbags.typesys import Stores, get_typestore

# 1. 配置你的数据包路径和标签
bags = {
    "Forward Euler": "exp_euler_0.02",
    "Heun (2nd Order)": "exp_heun_0.02",
    "Implicit Euler": "exp_implicit_0.02",
    "RK45 (Adaptive)": "exp_rk45_0.02",
    "RK4 (Ground Truth)": "exp_rk4_0.02" 
}

colors = {
    "Forward Euler": "red",
    "Heun (2nd Order)": "orange",
    "Implicit Euler": "green",
    "RK45 (Adaptive)": "purple",
    "RK4 (Ground Truth)": "black"
}

def read_odom_from_bag(bag_path):
    """从 rosbag 中提取时间、X、Y、Z 数据 (完美解决字典缺失问题)"""
    times, xs, ys, zs = [], [], [], []
    if not os.path.exists(bag_path):
        print(f"⚠️ 找不到数据包: {bag_path}")
        return None, None, None, None

    bag_dir = Path(bag_path)
    
    # 【核心修复】：显式加载 ROS 2 Humble 的标准消息类型库
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    # 将类型库强制传给 AnyReader
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/crazyflie/odom':
                msg = reader.deserialize(rawdata, connection.msgtype)
                # 核心修复：直接提取里程计消息内部的仿真时间，无视人为录制误差！
                sim_sec = msg.header.stamp.sec
                sim_nanosec = msg.header.stamp.nanosec
                sim_t = sim_sec + sim_nanosec * 1e-9
                times.append(sim_t)
                xs.append(msg.pose.pose.position.x)
                ys.append(msg.pose.pose.position.y)
                zs.append(msg.pose.pose.position.z)
        
    return np.array(times), np.array(xs), np.array(ys), np.array(zs)

# ================= 开始绘图 =================
# 自动兼容新老版本 Matplotlib 的学术风背景
try:
    plt.style.use('seaborn-v0_8-whitegrid')
except OSError:
    try:
        plt.style.use('seaborn-whitegrid')
    except OSError:
        plt.style.use('ggplot')

fig = plt.figure(figsize=(16, 6))

# --- 图 1：3D 空间轨迹对比 ---
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title("3D Trajectory Comparison (8-Figure Flight, h=0.02s)", fontsize=14)

# --- 图 2：Z轴高度时序对比 ---
ax2 = fig.add_subplot(122)
ax2.set_title("Z-Axis Altitude Tracking over Time", fontsize=14)

# 遍历所有算法的数据包进行绘制
for label, path in bags.items():
    t, x, y, z = read_odom_from_bag(path)
    if t is None or len(t) == 0: 
        continue
        
    linewidth = 2.5 if "RK4" in label else 1.5
    ax1.plot(x, y, z, label=label, color=colors[label], linewidth=linewidth, alpha=0.8)
    ax2.plot(t, z, label=label, color=colors[label], linewidth=linewidth, alpha=0.8)

# 装饰 3D 图
ax1.set_xlabel("X Position (m)")
ax1.set_ylabel("Y Position (m)")
ax1.set_zlabel("Z Position (m)")
ax1.legend()

# 装饰 2D 图
ax2.set_xlabel("Simulation Time (s)")
ax2.set_ylabel("Altitude Z (m)")
ax2.axhline(y=1.5, color='gray', linestyle='--', label="Target Altitude Base")
ax2.legend()

plt.tight_layout()
plt.show()
