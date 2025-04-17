import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# 加载增强数据集（包含速度、加速度）
data = pd.read_csv('pid_tracking_data.csv', 
                 skiprows=1,
                 names=[
                     'timestamp',
                     'actual_x', 'target_x',
                     'actual_y', 'target_y',
                     'actual_z', 'target_z',
                     'actual_vel_x', 'target_vel_x',
                     'actual_vel_y', 'target_vel_y',
                     'actual_vel_z', 'target_vel_z',
                     'actual_acc_x', 'target_acc_x',
                     'actual_acc_y', 'target_acc_y',
                     'actual_acc_z', 'target_acc_z'
                 ])

# 转换时间戳
data['time'] = data['timestamp'] - data['timestamp'].min()

# 创建3x3分析图表 (3轴 x 3参数)
plt.figure(figsize=(18, 25))

# 定义绘图参数
plot_config = {
    'x': {'row': 0, 'label': 'X'},
    'y': {'row': 1, 'label': 'Y'},
    'z': {'row': 2, 'label': 'Z'}
}

param_config = {
    'pos': {'col': 0, 'type': 'Position (m)'},
    'vel': {'col': 1, 'type': 'Velocity (m/s)'},
    'acc': {'col': 2, 'type': 'Acceleration (m/s²)'}
}

# 遍历三个轴生成子图
for axis in ['x', 'y', 'z']:
    row = plot_config[axis]['row']
    
    # ---------- 位置跟踪 ---------- 
    plt.subplot(3, 3, row*3 + 1)
    plt.plot(data['time'], data[f'actual_{axis}'], 'b-', label='Actual', linewidth=1.2)
    plt.plot(data['time'], data[f'target_{axis}'], 'r--', label='Target', linewidth=1)
    plt.fill_between(data['time'], data[f'actual_{axis}'], data[f'target_{axis}'],
                   where=(data[f'actual_{axis}'] > data[f'target_{axis}']),
                   facecolor='red', alpha=0.2, label='Positive Error')
    plt.fill_between(data['time'], data[f'actual_{axis}'], data[f'target_{axis}'],
                   where=(data[f'actual_{axis}'] <= data[f'target_{axis}']),
                   facecolor='green', alpha=0.2, label='Negative Error')
    plt.title(f"{plot_config[axis]['label']}-Axis Position Tracking")
    plt.ylabel(param_config['pos']['type'])
    plt.grid(True, alpha=0.3)
    if row == 0:
        plt.legend(loc='upper right')
    
    # ---------- 速度跟踪 ----------
    plt.subplot(3, 3, row*3 + 2)
    plt.plot(data['time'], data[f'actual_vel_{axis}'], 'g-', label='Actual Vel', linewidth=1.2)
    plt.plot(data['time'], data[f'target_vel_{axis}'], 'm--', label='Target Vel', linewidth=1)
    plt.title(f"{plot_config[axis]['label']}-Axis Velocity Tracking")
    plt.ylabel(param_config['vel']['type'])
    plt.grid(True, alpha=0.3)
    if row == 0:
        plt.legend()
    
    # ---------- 加速度跟踪 ----------
    plt.subplot(3, 3, row*3 + 3)
    plt.plot(data['time'], data[f'actual_acc_{axis}'], 'purple', label='Actual Acc', linewidth=1.2)
    plt.plot(data['time'], data[f'target_acc_{axis}'], 'orange', linestyle='--', label='Target Acc', linewidth=1)
    plt.title(f"{plot_config[axis]['label']}-Axis Acceleration Tracking")
    plt.ylabel(param_config['acc']['type'])
    plt.xlabel('Time (s)')
    plt.grid(True, alpha=0.3)
    if row == 0:
        plt.legend()

plt.tight_layout(pad=3.0)
plt.savefig('full_analysis.png', dpi=300)
plt.show()

# ---------- 增强性能指标 ----------
print("\nEnhanced Performance Metrics:")
for axis in ['x', 'y', 'z']:
    print(f"\n{axis.upper()}-Axis:")
    
    # 位置误差
    pos_error = data[f'actual_{axis}'] - data[f'target_{axis}']
    print("[Position]")
    print(f"  Max Overshoot: {pos_error.max():.4f} m")
    print(f"  Steady-state Error: {pos_error.iloc[-100:].mean():.4f} m")
    
    # 速度误差
    vel_error = data[f'actual_vel_{axis}'] - data[f'target_vel_{axis}']
    print("\n[Velocity]")
    print(f"  RMS Error: {np.sqrt((vel_error**2).mean()):.4f} m/s")
    print(f"  Max Deviation: {vel_error.abs().max():.4f} m/s")
    
    # 加速度误差
    acc_error = data[f'actual_acc_{axis}'] - data[f'target_acc_{axis}']
    print("\n[Acceleration]")
    print(f"  Peak Error: {acc_error.abs().max():.4f} m/s²")
    print(f"  Mean Absolute Error: {acc_error.abs().mean():.4f} m/s²")