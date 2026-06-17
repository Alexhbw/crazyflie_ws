import os
import numpy as np
from rosbags.highlevel import AnyReader
from pathlib import Path
from rosbags.typesys import Stores, get_typestore

# 【修改这里】：换成你刚刚录制的新 bag 的路径！
bag_path = "/home/alex/crazyflie_ws/exp_euler_0.02_with_target" 

odom_topic = '/crazyflie/odom'
target_topic = '/crazyflie/target' 

times, real_xs, real_zs, target_xs, target_zs = [], [], [], [], []
typestore = get_typestore(Stores.ROS2_HUMBLE)

latest_target_x = 0.0
latest_target_z = 1.5 

with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
    for connection, timestamp, rawdata in reader.messages():
        
        # 1. 抓取目标值 (缓存最新状态)
        if connection.topic == target_topic:
            msg = reader.deserialize(rawdata, connection.msgtype)
            latest_target_x = msg.pose.position.x
            latest_target_z = msg.pose.position.z
            
        # 2. 抓取真实值 (并与最新的目标值强行绑定同一时间点)
        elif connection.topic == odom_topic:
            msg = reader.deserialize(rawdata, connection.msgtype)
            times.append(timestamp / 1e9)
            
            # 真实位置
            real_xs.append(msg.pose.pose.position.x)
            real_zs.append(msg.pose.pose.position.z)
            
            # 对齐的目标位置
            target_xs.append(latest_target_x)
            target_zs.append(latest_target_z)

# 保存为 CSV
if times:
    t0 = times[0]
    times = [t - t0 for t in times]
    data = np.column_stack((times, real_xs, real_zs, target_xs, target_zs))
    
    csv_name = "cpp_trajectory_with_target.csv"
    np.savetxt(csv_name, data, delimiter=",", header="t,real_x,real_z,target_x,target_z", comments="")
    print(f"✅ 成功导出完美的 5 列对齐数据: {csv_name}")