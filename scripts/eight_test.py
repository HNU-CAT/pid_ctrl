import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3, PoseStamped
from nav_msgs.msg import Path
from tracking_controller.msg import PositionCommand

class Figure8TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('figure8_trajectory_publisher')
        
        # 轨迹参数配置
        self.a = rospy.get_param('~x_amplitude', 8.0)   # X轴振幅
        self.b = rospy.get_param('~y_amplitude', 4.0)   # Y轴振幅
        self.omega = rospy.get_param('~angular_velocity', 0.5)  # 角速度(rad/s)
        self.phase = rospy.get_param('~phase_shift', np.pi/2)   # 相位差
        self.takeoff_height = rospy.get_param('~takeoff_height', 1.0)
        self.max_path_length = rospy.get_param('~max_path_length', 1000)
        self.yaw_control = rospy.get_param('~yaw_control', True)  # 航向控制开关
        
        # ROS接口初始化
        self.pub_cmd = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        self.pub_ref = rospy.Publisher('/reference_trajectory', Path, queue_size=10)
        
        # 运动参数
        self.rate = 100  # 控制频率
        self.dt = 1.0 / self.rate
        self.t = 0.0     # 时间累积量
        
        # 轨迹存储
        self.reference_path = Path()
        self.reference_path.header.frame_id = 'world'
        
        # 定时器启动
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.trajectory_update)

    def trajectory_update(self, event):
        """核心轨迹生成算法"""
        # 参数方程定义
        x = self.a * np.sin(self.omega * self.t)
        y = self.b * np.sin(2 * self.omega * self.t + self.phase)
        
        # 微分计算速度
        vx = self.a * self.omega * np.cos(self.omega * self.t)
        vy = 2 * self.b * self.omega * np.cos(2 * self.omega * self.t + self.phase)
        
        # 微分计算加速度
        ax = -self.a * (self.omega**2) * np.sin(self.omega * self.t)
        ay = -4 * self.b * (self.omega**2) * np.sin(2 * self.omega * self.t + self.phase)
        
        # 航向角计算
        yaw = np.arctan2(vy, vx) if self.yaw_control else 0.0
        
        # 发布控制指令
        self.publish_command(x, y, vx, vy, ax, ay, yaw)
        
        # 记录轨迹点
        self.record_trajectory(x, y)
        
        self.t += self.dt  # 时间累积

    def publish_command(self, x, y, vx, vy, ax, ay, yaw):
        """构造PositionCommand消息"""
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.position = Point(x, y, self.takeoff_height)
        msg.velocity = Vector3(vx, vy, 0)
        msg.acceleration = Vector3(ax, ay, 0)
        msg.yaw = yaw
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub_cmd.publish(msg)

    def record_trajectory(self, x, y):
        """轨迹可视化处理"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'
        pose.pose.position = Point(x, y, self.takeoff_height)
        
        self.reference_path.poses.append(pose)
        
        # 限制轨迹长度
        if len(self.reference_path.poses) > self.max_path_length:
            self.reference_path.poses.pop(0)
            
        self.reference_path.header.stamp = rospy.Time.now()
        self.pub_ref.publish(self.reference_path)

if __name__ == '__main__':
    try:
        Figure8TrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass