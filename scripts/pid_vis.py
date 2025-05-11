#!/usr/bin/env python3
"""
PID控制器性能可视化分析工具
功能：读取跟踪数据CSV文件，生成包含位置、速度、加速度和偏航角跟踪性能的可视化图表
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_data(file_path):
    """
    加载并预处理PID跟踪数据
    参数：
        file_path: CSV文件路径
    返回：
        处理后的DataFrame对象
    """
    # 定义列名（与CSV文件表头严格对应）
    columns = [
        'timestamp',         # 时间戳
        'actual_x',          # X轴实际位置
        'target_x',          # X轴目标位置
        'actual_yaw',        # 实际偏航角
        'actual_y',          # Y轴实际位置
        'target_y',          # Y轴目标位置
        'target_yaw',        # 目标偏航角
        'actual_z',          # Z轴实际位置
        'target_z',          # Z轴目标位置
        'actual_vel_x',      # X轴实际速度
        'target_vel_x',      # X轴目标速度
        'actual_vel_y',      # Y轴实际速度
        'target_vel_y',      # Y轴目标速度
        'actual_vel_z',      # Z轴实际速度
        'target_vel_z',      # Z轴目标速度
        'actual_acc_x',      # X轴实际加速度
        'target_acc_x',      # X轴目标加速度
        'actual_acc_y',      # Y轴实际加速度
        'target_acc_y',      # Y轴目标加速度
        'actual_acc_z',      # Z轴实际加速度
        'target_acc_z'       # Z轴目标加速度
    ]
    
    # 读取CSV文件
    data = pd.read_csv(file_path, skiprows=1, names=columns)
    
    # 时间标准化处理（从0开始）
    data['time'] = data['timestamp'] - data['timestamp'].min()
    
    # 角度归一化处理（将角度限制在[-π, π]）
    data['actual_yaw'] = np.arctan2(np.sin(data['actual_yaw']), np.cos(data['actual_yaw']))
    data['target_yaw'] = np.arctan2(np.sin(data['target_yaw']), np.cos(data['target_yaw']))
    
    return data

def plot_performance(data):
    """
    生成性能分析图表
    参数：
        data: 包含跟踪数据的DataFrame
    """
    # 创建4行3列的子图布局 (新增yaw分析行)
    fig = plt.figure(figsize=(18, 30))
    fig.suptitle('Comprehensive Analysis of PID Control Performance', fontsize=20, y=0.95)
    
    # 定义子图布局参数
    axis_config = {
        'x': {'row': 0, 'label': 'X-Axis'},
        'y': {'row': 1, 'label': 'Y-Axis'},
        'z': {'row': 2, 'label': 'Z-Axis'},
        'yaw': {'row': 3, 'label': 'Yaw'}
    }

    
    # 遍历各轴生成图表
    for axis in ['x', 'y', 'z', 'yaw']:
        row = axis_config[axis]['row']
        
        # 位置跟踪图表（所有轴共用相同结构）
        plt.subplot(4, 3, row*3 + 1)
        if axis != 'yaw':
            plot_position(axis, data, axis_config)
        else:
            plot_yaw_tracking(data)
        
        # 速度跟踪图表（yaw角不需要速度跟踪）
        if axis != 'yaw':
            plt.subplot(4, 3, row*3 + 2)
            plot_velocity(axis, data,axis_config)
        
        # 加速度跟踪图表（yaw角不需要加速度跟踪）
        if axis != 'yaw':
            plt.subplot(4, 3, row*3 + 3)
            plot_acceleration(axis, data,axis_config)
        else:
            plt.subplot(4, 3, row*3 + 3)
            plot_yaw_error_distribution(data)
    
    plt.tight_layout(pad=3.0)
    plt.savefig('full_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_position(axis, data,axis_config):
    """位置跟踪可视化"""
    plt.title(f"{axis_config[axis]['label']} Position Tracking") 
    plt.plot(data['time'], data[f'actual_{axis}'], 'b-', label='Actual', linewidth=1.2)
    plt.plot(data['time'], data[f'target_{axis}'], 'r--', label='Target', linewidth=1)
    plt.fill_between(data['time'], data[f'actual_{axis}'], data[f'target_{axis}'],
                    where=(data[f'actual_{axis}'] > data[f'target_{axis}']),
                    facecolor='red', alpha=0.2, label='Target')
    plt.fill_between(data['time'], data[f'actual_{axis}'], data[f'target_{axis}'],
                    where=(data[f'actual_{axis}'] <= data[f'target_{axis}']),
                    facecolor='green', alpha=0.2, label='Negative Error')
    plt.ylabel('Position (m)')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right')

def plot_velocity(axis, data, axis_config):
    """速度跟踪可视化"""
    plt.title(f"{axis_config[axis]['label']} Velocity Tracking")
    plt.plot(data['time'], data[f'actual_vel_{axis}'], 'g-', label='Actual', linewidth=1.2)
    plt.plot(data['time'], data[f'target_vel_{axis}'], 'm--', label='Target', linewidth=1)
    plt.ylabel('Velocity (m/s)')
    plt.grid(True, alpha=0.3)
    plt.legend()

def plot_acceleration(axis, data, axis_config):
    """加速度跟踪可视化"""
    plt.title(f"{axis_config[axis]['label']}Acceleration Tracking")
    plt.plot(data['time'], data[f'actual_acc_{axis}'], 'purple', label='Actual', linewidth=1.2)
    plt.plot(data['time'], data[f'target_acc_{axis}'], 'orange', linestyle='--', label='Target', linewidth=1)
    plt.ylabel('Acceleration (m/s²)')
    plt.grid(True, alpha=0.3)
    plt.legend()

def plot_yaw_tracking(data):
    """偏航角跟踪可视化"""
    plt.plot(data['time'], data['actual_yaw'], 'b-', label='Actual', linewidth=1.5)
    plt.plot(data['time'], data['target_yaw'], 'r--', label='Target', linewidth=1.2)
    plt.title("Yaw Angle Tracking Performance")
    plt.ylabel('Angle (rad)')
    plt.grid(True, alpha=0.3)
    plt.legend()

def plot_yaw_error_distribution(data):
    """偏航角误差分布可视化"""
    yaw_error = np.abs(data['actual_yaw'] - data['target_yaw'])
    plt.hist(yaw_error, bins=50, color='purple', alpha=0.7)
    plt.title("Yaw Error Distribution")
    plt.xlabel('Error (rad)')
    plt.ylabel('Frequency')
    plt.grid(True, alpha=0.3)

def print_metrics(data):
    """打印性能指标"""
    print("\nComprehensive Performance Metrics:")
    
    axis_config = {
        'x': {'row': 0, 'label': 'X-Axis'},
        'y': {'row': 1, 'label': 'Y-Axis'},
        'z': {'row': 2, 'label': 'Z-Axis'},
        'yaw': {'row': 3, 'label': 'Yaw'}
    }
    # 公共性能指标分析
    for axis in ['x', 'y', 'z', 'yaw']:
        print(f"\n{axis_config[axis]['label']}性能:")
        
        if axis != 'yaw':
            # 位置误差分析
            pos_error = data[f'actual_{axis}'] - data[f'target_{axis}']
            print("[位置]")
            print(f"  最大超调量: {pos_error.max():.4f} m")
            print(f"  稳态误差: {pos_error.iloc[-100:].mean():.4f} m")

            # 速度误差分析
            vel_error = data[f'actual_vel_{axis}'] - data[f'target_vel_{axis}']
            print("\n[速度]")
            print(f"  RMS误差: {np.sqrt((vel_error**2).mean()):.4f} m/s")
            print(f"  最大偏差: {vel_error.abs().max():.4f} m/s")

            # 加速度误差分析
            acc_error = data[f'actual_acc_{axis}'] - data[f'target_acc_{axis}']
            print("\n[加速度]")
            print(f"  峰值误差: {acc_error.abs().max():.4f} m/s²")
            print(f"  平均绝对误差: {acc_error.abs().mean():.4f} m/s²")
        else:
            # 偏航角误差分析
            yaw_error = data['actual_yaw'] - data['target_yaw']
            print("[偏航角]")
            print(f"  最大绝对误差: {np.abs(yaw_error).max():.4f} rad")
            print(f"  平均绝对误差: {np.abs(yaw_error).mean():.4f} rad")
            print(f"  RMS误差: {np.sqrt((yaw_error**2).mean()):.4f} rad")

if __name__ == '__main__':
    # 数据加载
    data = load_data('pid_tracking_data.csv')
    
    # 生成可视化图表
    plot_performance(data)
    
    # 输出性能指标
    print_metrics(data)