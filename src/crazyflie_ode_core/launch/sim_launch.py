import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('crazyflie_ode_core')
    urdf_file = os.path.join(pkg_path, 'urdf', 'crazyflie.urdf')
    
    # 路径拼接：告诉 RViz 配置文件的具体位置
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'sim.rviz')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 声明可接收的参数
    solver_arg = DeclareLaunchArgument('solver', default_value='rk4')
    step_arg = DeclareLaunchArgument('step_size', default_value='0.01')

    return LaunchDescription([
        solver_arg,
        step_arg,
        Node(
            package='crazyflie_ode_core',
            executable='ode_engine_node',
            parameters=[{
                'use_sim_time': True,
                'solver': LaunchConfiguration('solver'),
                'step_size': LaunchConfiguration('step_size')
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            # 【修复点】：这一行告诉 RViz 启动时自动加载你刚刚保存的 sim.rviz 文件
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        )
    ])