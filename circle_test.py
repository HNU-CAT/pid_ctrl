import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
from tracking_controller.msg import PositionCommand
from geometry_msgs.msg import PoseStamped

def normalize_yaw(yaw):
    yaw = yaw % (2 * np.pi)
    return yaw - 2 * np.pi if yaw > np.pi else yaw

class CircleTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('circle_trajectory_publisher')
        
        # 参数初始化
        self.radius = rospy.get_param('~circle_radius', 3.0)
        self.time_step = rospy.get_param('~time_to_max_radius', 30.0)
        self.yaw_control = rospy.get_param('~yaw_control', True)
        self.velocity = rospy.get_param('~velocity', 2.5)
        self.takeoff_height = rospy.get_param('~takeoff_height', 1.0)
        self.max_path_length = rospy.get_param('~max_path_length', 600)
        
        # 初始化PID参数
        self.Kp_yaw = 0.5
        self.Ki_yaw = 0.0
        self.Kd_yaw = 0.1
        self.last_yaw_error = 0.0
        self.integral_yaw_error = 0.0
        
        # 订阅当前无人机状态
        self.current_yaw = 0.0
        self.odom_sub = rospy.Subscriber('/odometry', Odometry, self.odom_callback)
        
        # 发布器
        self.pub_cmd = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        self.pub_ref = rospy.Publisher('/reference_trajectory', Path, queue_size=10)
        
        self.rate = 100
        self.dt = 1.0 / self.rate
        self.theta = 0.0
        self.current_radius = 0.0
        self.current_velocity = 0.0
        self.step = self.time_step * self.rate
        self.circle_stage = 1
        self.start_time = rospy.Time.now()
        
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'world'
        self.reference_path = Path()
        self.reference_path.header.frame_id = 'world'
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
    
    def odom_callback(self, msg):
        # 解析当前偏航角
        q = msg.pose.pose.orientation
        self.current_yaw = normalize_yaw(np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2)))
    
    def compute_yaw_rate(self, target_yaw):
        error = normalize_yaw(target_yaw - self.current_yaw)
        self.integral_yaw_error += error * self.dt
        derivative = (error - self.last_yaw_error) / self.dt
        yaw_rate = self.Kp_yaw * error + self.Ki_yaw * self.integral_yaw_error + self.Kd_yaw * derivative
        yaw_rate = np.clip(yaw_rate, -1.0, 1.0)
        self.last_yaw_error = error
        return yaw_rate
    
    def timer_callback(self, event):
        t = (rospy.Time.now() - self.start_time).to_sec()
        if self.circle_stage == 1:
            self.current_radius += self.radius / self.step
            self.current_velocity += self.velocity / self.step
            if abs(self.current_radius - self.radius) <= 0.01:
                self.circle_stage = 2
                self.start_theta = self.theta
        elif self.circle_stage == 2:
            self.current_radius = self.radius
            self.current_velocity = self.velocity
            if abs((self.theta - self.start_theta) - (6 * np.pi)) <= 0.1:
                self.circle_stage = 3
        elif self.circle_stage == 3:
            self.current_radius -= self.radius / self.step
            self.current_velocity -= self.velocity / self.step
            if self.current_radius <= 0.01:
                rospy.loginfo("Circle trajectory completed.")
                rospy.signal_shutdown("Trajectory Finished")
                return
        self.theta = self.current_velocity / self.current_radius * t
        x = self.current_radius * np.cos(self.theta)
        y = self.current_radius * np.sin(self.theta)
        vx = -self.current_velocity * np.sin(self.theta)
        vy = self.current_velocity * np.cos(self.theta)
        ax = -self.current_velocity**2 / self.current_radius * np.cos(self.theta)
        ay = -self.current_velocity**2 / self.current_radius * np.sin(self.theta)
        
        # 计算并归一化yaw
        yaw = self.theta + np.pi / 2 if self.yaw_control else 0.0
        
        # 计算yaw_rate
        yaw_rate = self.compute_yaw_rate(yaw) if self.yaw_control else 0.0
        
        self.publish_position_command(x, y, self.takeoff_height, vx, vy, 0, ax, ay, 0, yaw)
        self.publish_reference_trajectory(x, y, self.takeoff_height)
    
    def publish_position_command(self, x, y, z, vx, vy, vz, ax, ay, az, yaw):
        msg = PositionCommand()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='world')
        msg.position = Point(x, y, z)
        msg.velocity = Vector3(vx, vy, vz)
        msg.acceleration = Vector3(ax, ay, az)
        msg.yaw = yaw  # 不再直接设置目标yaw角
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub_cmd.publish(msg)
    
    def publish_reference_trajectory(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'
        pose.pose.position = Point(x, y, z)
        self.reference_path.poses.append(pose)
        if len(self.reference_path.poses) > self.max_path_length:
            self.reference_path.poses.pop(0)
        self.reference_path.header.stamp = rospy.Time.now()
        self.pub_ref.publish(self.reference_path)

if __name__ == '__main__':
    try:
        CircleTrajectoryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass