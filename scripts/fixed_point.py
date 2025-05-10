#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tracking_controller.msg import PositionCommand
from geometry_msgs.msg import PoseStamped

class FixedPointPublisher:
    def __init__(self):
        rospy.init_node('fixed_point_publisher')
        
        # 读取固定点参数
        self.fixed_x = rospy.get_param('~x', 2)
        self.fixed_y = rospy.get_param('~y', 0.0)
        self.fixed_z = rospy.get_param('~z', 1.0)
        self.yaw_control = rospy.get_param('~yaw_control', False)
        self.fixed_yaw = rospy.get_param('~yaw', 0.0)
        self.max_path_length = rospy.get_param('~max_path_length', 600)

        # 初始化发布器
        self.pub_cmd = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        self.pub_ref = rospy.Publisher('/reference_trajectory', Path, queue_size=10)

        # 初始化路径
        self.reference_path = Path()
        self.reference_path.header.frame_id = 'world'

        # 设置定时器（100Hz）
        self.rate = 100
        self.dt = 1.0 / self.rate
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)

    def timer_callback(self, event):
        # 发布固定位置指令
        self.publish_position_command(
            self.fixed_x, 
            self.fixed_y, 
            self.fixed_z,
            yaw=self.fixed_yaw if self.yaw_control else 0.0
        )
        
        # 更新参考轨迹
        self.publish_reference_trajectory(
            self.fixed_x,
            self.fixed_y,
            self.fixed_z
        )

    def publish_position_command(self, x, y, z, yaw=0.0):
        msg = PositionCommand()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='world')
        msg.position = Point(x, y, z)
        msg.velocity = Vector3(0.0, 0.0, 0.0)    # 固定点速度为0
        msg.acceleration = Vector3(0.0, 0.0, 0.0) # 加速度为0
        msg.yaw = yaw
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub_cmd.publish(msg)

    def publish_reference_trajectory(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'
        pose.pose.position = Point(x, y, z)
        
        # 保持路径长度不超过限制
        self.reference_path.poses.append(pose)
        if len(self.reference_path.poses) > self.max_path_length:
            self.reference_path.poses.pop(0)
            
        self.reference_path.header.stamp = rospy.Time.now()
        self.pub_ref.publish(self.reference_path)

if __name__ == '__main__':
    try:
        FixedPointPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass