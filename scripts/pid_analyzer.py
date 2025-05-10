#!/usr/bin/env python3
import rospy
import numpy as np
import os
import threading
from collections import defaultdict
from nav_msgs.msg import Odometry
from tracking_controller.msg import PositionCommand
from message_filters import ApproximateTimeSynchronizer, Subscriber

class EnhancedPIDDataRecorder:
    def __init__(self):
        rospy.init_node('enhanced_pid_data_recorder')
        
        # 初始化参数和路径
        save_path_name = rospy.get_param("controller/save_path", "~/experment/pid_ctrl/pid_tracking_data.csv")
        self.odom_topic = rospy.get_param("controller/odom_topic", "/mavros/vision_odom/odom")
        self.cmd_topic = rospy.get_param("controller/cmd_topic", "/planning/pos_cmd")
        
        # 创建保存目录
        self.save_path = os.path.expanduser(save_path_name)
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        rospy.loginfo(f"数据将保存至: {self.save_path}")

        # 初始化数据结构
        self.data = defaultdict(list)
        self.data_lock = threading.Lock()
        self.prev_vel = np.zeros(3)
        self.prev_time = None
        self.saving_flag = False
        self.data_count = 0

        # 初始化订阅器
        self._init_subscribers()
        
        # 注册关闭钩子
        rospy.on_shutdown(self.shutdown_hook)

    def _init_subscribers(self):
        """初始化消息订阅器"""
        rospy.loginfo(f"订阅话题: \n- 里程计: {self.odom_topic}\n- 控制指令: {self.cmd_topic}")
        
        try:
            odom_sub = Subscriber(self.odom_topic, Odometry)
            target_sub = Subscriber(self.cmd_topic, PositionCommand)
            
            self.ts = ApproximateTimeSynchronizer([odom_sub, target_sub],
                                                queue_size=30,
                                                slop=0.05)
            self.ts.registerCallback(self.sync_callback)
        except Exception as e:
            rospy.logerr(f"订阅器初始化失败: {str(e)}")
            raise

    def sync_callback(self, odom_msg, target_msg):
        """带异常处理的同步回调"""
        if self.saving_flag:
            return
            
        try:
            timestamp = odom_msg.header.stamp.to_sec()
            
            with self.data_lock:
                # 记录基础数据
                self._record_basic_data(timestamp, odom_msg, target_msg)
                
                # 记录实际速度
                self._record_actual_velocity(odom_msg)
                
                # 记录目标参数
                self._record_target_dynamics(target_msg)
                
                # 计算加速度
                self._calculate_actual_acceleration(timestamp)
                
                self.data_count += 1
                
        except Exception as e:
            rospy.logerr(f"回调处理异常: {str(e)}")
            self._emergency_save()

    def _record_basic_data(self, t, odom, target):
        """记录基础位置信息"""
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
        """记录目标动态参数"""
        self.data['target_vel_x'].append(target.velocity.x)
        self.data['target_vel_y'].append(target.velocity.y)
        self.data['target_vel_z'].append(target.velocity.z)
        self.data['target_acc_x'].append(target.acceleration.x)
        self.data['target_acc_y'].append(target.acceleration.y)
        self.data['target_acc_z'].append(target.acceleration.z)

    def _calculate_actual_acceleration(self, current_time):
        """计算实际加速度"""
        try:
            if self.prev_time is None:
                self.prev_time = current_time
                return

            dt = current_time - self.prev_time
            if dt <= 0:
                return

            current_vel = np.array([
                self.data['actual_vel_x'][-1],
                self.data['actual_vel_y'][-1],
                self.data['actual_vel_z'][-1]
            ])
            
            acc = (current_vel - self.prev_vel) / dt
            self.data['actual_acc_x'].append(acc[0])
            self.data['actual_acc_y'].append(acc[1])
            self.data['actual_acc_z'].append(acc[2])
            
            self.prev_vel = current_vel
            self.prev_time = current_time
            
        except IndexError:
            self.data['actual_acc_x'].append(0.0)
            self.data['actual_acc_y'].append(0.0)
            self.data['actual_acc_z'].append(0.0)

    def shutdown_hook(self):
        """关闭时的保存钩子"""
        if not self.saving_flag and self.data_count > 0:
            rospy.logwarn("接收到关闭信号，正在保存数据...")
            self.save_data()

    def _emergency_save(self):
        """异常情况下的紧急保存"""
        rospy.logerr("触发紧急保存机制")
        self.save_data()

    def save_data(self):
        """线程安全的数据保存方法"""
        if self.saving_flag or self.data_count == 0:
            return
            
        self.saving_flag = True
        rospy.loginfo("开始数据保存流程...")
        
        try:
            with self.data_lock:
                # 检查数据有效性
                base_length = len(self.data['timestamp'])
                if base_length == 0:
                    rospy.logwarn("无有效数据需要保存")
                    return
                
                # 对齐所有数据长度
                min_length = min([
                    len(self.data[key]) 
                    for key in self.data.keys()
                ] + [base_length])
                
                # 构建数据列
                columns = [
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
                ]
                
                # 验证数据完整性
                data_to_save = []
                for col in columns:
                    if len(self.data[col]) < min_length:
                        rospy.logwarn(f"字段 {col} 数据长度不足，自动截断")
                    data_to_save.append(self.data[col][:min_length])
                
                # 转换为numpy数组
                data_array = np.column_stack(data_to_save)
                
                # 生成文件头
                header = ",".join(columns)
                
                # 保存文件
                np.savetxt(self.save_path, data_array,
                         header=header, delimiter=',', fmt='%.6f')
                         
                rospy.loginfo(f"成功保存 {min_length} 条数据 (总接收 {self.data_count} 条)")
                
        except Exception as e:
            rospy.logerr(f"保存过程中发生错误: {str(e)}")
        finally:
            self.saving_flag = False

    def run(self):
        """带定期保存的主循环"""
        # 每1分钟自动保存一次
        rospy.Timer(rospy.Duration(60), lambda _: self.save_data(), oneshot=False)
        rospy.spin()

if __name__ == '__main__':
    recorder = None
    try:
        recorder = EnhancedPIDDataRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        if recorder:
            recorder.save_data()
    except Exception as e:
        rospy.logerr(f"程序异常终止: {str(e)}")
        if recorder:
            recorder._emergency_save()
    finally:
        if recorder:
            recorder.save_data()