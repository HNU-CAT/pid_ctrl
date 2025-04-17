#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tracking_controller.msg import PositionCommand
from message_filters import ApproximateTimeSynchronizer, Subscriber

class EnhancedPIDDataRecorder:
    def __init__(self):
        rospy.init_node('enhanced_pid_data_recorder')
        
        # 初始化消息同步器
        odom_sub = Subscriber("/iris_0/mavros/vision_odom/odom", Odometry)
        target_sub = Subscriber("/planning/pos_cmd", PositionCommand)
        
        self.ts = ApproximateTimeSynchronizer([odom_sub, target_sub],
                                            queue_size=30,
                                            slop=0.05)
        self.ts.registerCallback(self.sync_callback)
        
        # 扩展数据存储结构
        self.data = {
            # 时间戳
            'timestamp': [],
            # 位置
            'actual_x': [], 'target_x': [],
            'actual_y': [], 'target_y': [],
            'actual_z': [], 'target_z': [],
            # 速度
            'actual_vel_x': [], 'target_vel_x': [],
            'actual_vel_y': [], 'target_vel_y': [],
            'actual_vel_z': [], 'target_vel_z': [],
            # 加速度
            'actual_acc_x': [], 'target_acc_x': [],
            'actual_acc_y': [], 'target_acc_y': [],
            'actual_acc_z': [], 'target_acc_z': []
        }
        
        # 用于加速度计算的前一时刻数据
        self.prev_vel = np.zeros(3)
        self.prev_time = None
        
        rospy.on_shutdown(self.save_data)

    def sync_callback(self, odom_msg, target_msg):
        """同步处理传感器数据和命令数据"""
        timestamp = odom_msg.header.stamp.to_sec()
        
        # 记录基础数据
        self._record_basic_data(timestamp, odom_msg, target_msg)
        
        # 记录实际速度
        self._record_actual_velocity(odom_msg)
        
        # 记录目标速度和加速度
        self._record_target_dynamics(target_msg)
        
        # 计算并记录实际加速度
        self._calculate_actual_acceleration(timestamp)

    def _record_basic_data(self, t, odom, target):
        """记录位置信息"""
        self.data['timestamp'].append(t)
        # 实际位置
        self.data['actual_x'].append(odom.pose.pose.position.x)
        self.data['actual_y'].append(odom.pose.pose.position.y)
        self.data['actual_z'].append(odom.pose.pose.position.z)
        # 目标位置
        self.data['target_x'].append(target.position.x)
        self.data['target_y'].append(target.position.y)
        self.data['target_z'].append(target.position.z)

    def _record_actual_velocity(self, odom):
        """记录实际速度"""
        self.data['actual_vel_x'].append(odom.twist.twist.linear.x)
        self.data['actual_vel_y'].append(odom.twist.twist.linear.y)
        self.data['actual_vel_z'].append(odom.twist.twist.linear.z)

    def _record_target_dynamics(self, target):
        """记录目标动力学参数"""
        self.data['target_vel_x'].append(target.velocity.x)
        self.data['target_vel_y'].append(target.velocity.y)
        self.data['target_vel_z'].append(target.velocity.z)
        
        self.data['target_acc_x'].append(target.acceleration.x)
        self.data['target_acc_y'].append(target.acceleration.y)
        self.data['target_acc_z'].append(target.acceleration.z)

    def _calculate_actual_acceleration(self, current_time):
        """计算实际加速度"""
        if self.prev_time is not None and len(self.data['timestamp']) >= 2:
            dt = current_time - self.prev_time
            if dt > 0:
                current_vel = np.array([
                    self.data['actual_vel_x'][-1],
                    self.data['actual_vel_y'][-1],
                    self.data['actual_vel_z'][-1]
                ])
                acc = (current_vel - self.prev_vel) / dt
                
                self.data['actual_acc_x'].append(acc[0])
                self.data['actual_acc_y'].append(acc[1])
                self.data['actual_acc_z'].append(acc[2])
                return
                
        # 首帧数据或无效时间间隔处理
        self.data['actual_acc_x'].append(0.0)
        self.data['actual_acc_y'].append(0.0)
        self.data['actual_acc_z'].append(0.0)
        
        # 更新前一时序数据
        self.prev_vel = np.array([
            self.data['actual_vel_x'][-1],
            self.data['actual_vel_y'][-1],
            self.data['actual_vel_z'][-1]
        ])
        self.prev_time = current_time

    def save_data(self):
        """保存增强数据集"""
        # 对齐数据长度
        min_length = min(len(self.data['timestamp']), 
                       len(self.data['actual_acc_x']))
        
        data_array = np.column_stack([
            self.data['timestamp'][:min_length],
            # 位置
            self.data['actual_x'][:min_length], self.data['target_x'][:min_length],
            self.data['actual_y'][:min_length], self.data['target_y'][:min_length],
            self.data['actual_z'][:min_length], self.data['target_z'][:min_length],
            # 速度
            self.data['actual_vel_x'][:min_length], self.data['target_vel_x'][:min_length],
            self.data['actual_vel_y'][:min_length], self.data['target_vel_y'][:min_length],
            self.data['actual_vel_z'][:min_length], self.data['target_vel_z'][:min_length],
            # 加速度
            self.data['actual_acc_x'][:min_length], self.data['target_acc_x'][:min_length],
            self.data['actual_acc_y'][:min_length], self.data['target_acc_y'][:min_length],
            self.data['actual_acc_z'][:min_length], self.data['target_acc_z'][:min_length]
        ])
        
        header = (
            "timestamp,"
            "actual_x,target_x,actual_y,target_y,actual_z,target_z,"
            "actual_vel_x,target_vel_x,actual_vel_y,target_vel_y,actual_vel_z,target_vel_z,"
            "actual_acc_x,target_acc_x,actual_acc_y,target_acc_y,actual_acc_z,target_acc_z"
        )
        
        np.savetxt('pid_tracking_data.csv', data_array,
                 header=header, delimiter=',', fmt='%.6f')
        rospy.loginfo(f"保存增强数据集，包含 {min_length} 个有效数据点")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    recorder = EnhancedPIDDataRecorder()
    try:
        recorder.run()
    except Exception as e:
        rospy.logerr(f"程序异常: {str(e)}")
    finally:
        recorder.save_data()