import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tracking_controller.msg import PositionCommand
import nav_msgs.msg
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, SetModeRequest
import tf

def normalize_yaw(yaw):
    yaw = yaw % (2 * np.pi)
    return yaw - 2 * np.pi if yaw > np.pi else yaw

class CircleTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('circle_trajectory_publisher')
        
        # 参数初始化
        self.radius = rospy.get_param('controller/circle_radius', 3.0)
        self.time_step = rospy.get_param('controller/time_to_max_radius', 30.0)
        self.yaw_control = rospy.get_param('controller/yaw_control', True)
        self.velocity = rospy.get_param('controller/velocity', 2.5)
        self.takeoff_height = rospy.get_param('controller/takeoff_height', 1.0)
        self.max_path_length = rospy.get_param('controller/max_path_length', 600)
        self.z_amplitude = rospy.get_param('controller/z_amplitude', 0.5)
        self.z_frequency = rospy.get_param('controller/z_frequency', 0.2)
        self.takeoff_threshold = 0.1  # 起飞位置误差阈值
        
        # 话题与服务（适配iris_0命名空间）
        cmd_topic = rospy.get_param("controller/planning/pos_cmd", "/planning/pos_cmd")
        self.pub_cmd = rospy.Publisher(cmd_topic, PositionCommand, queue_size=10)
        self.pub_ref = rospy.Publisher('/reference_trajectory', Path, queue_size=10)
        self.pub_setpoint = rospy.Publisher('/iris_0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # MAVROS服务（等待服务可用后创建客户端）
        rospy.wait_for_service('/iris_0/mavros/cmd/arming')
        self.arm_client = rospy.ServiceProxy('/iris_0/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/iris_0/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/iris_0/mavros/set_mode', SetMode)
        
        # 订阅状态和里程计
        self.sub_state = rospy.Subscriber('/iris_0/mavros/state', State, self.state_callback)
        self.sub_odom = rospy.Subscriber('/iris_0/mavros/local_position/odom', nav_msgs.msg.Odometry, self.odom_callback)
        self.current_state = State()
        self.curr_pos = Point(0, 0, 0)
        
        # 状态变量
        self.armed = False
        self.offboard = False
        self.takeoff_complete = False
        self.state = 0  # 0-初始化前准备，1-初始化（模式切换），2-起飞，3-圆形轨迹
        self.last_req = rospy.Time.now()  # 服务请求时间戳
        self.start_time = rospy.Time.now()
        self.curr_yaw = 0.0 
        self.first_init = True
        
        # 轨迹参数
        self.rate = 100
        self.dt = 1.0 / self.rate
        self.theta = 0.0
        self.current_radius = 0.0
        self.current_velocity = 0.0
        self.step = self.time_step * self.rate
        self.z_phase = 0.0
        
        # 路径可视化
        self.reference_path = Path()
        self.reference_path.header.frame_id = 'world'
        
        # 提前发送100个初始指令（满足PX4 OFFBOARD模式要求）
        self.send_initial_setpoints()
        
        # 启动定时器
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
    
    def send_initial_setpoints(self):
        """提前发送100个初始位置指令，确保OFFBOARD模式可切换"""
        rospy.loginfo("发送初始位置指令（准备切换OFFBOARD模式）...")
        for i in range(100):
            setpoint = PoseStamped()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.header.frame_id = "world"
            setpoint.pose.position = self.curr_pos  # 初始位置为当前位置
            self.pub_setpoint.publish(setpoint)
            rospy.sleep(0.05)  # 约20Hz发送
    
    def state_callback(self, msg):
        self.current_state = msg
        self.armed = msg.armed
        self.offboard = (msg.mode == "OFFBOARD")
    
    def odom_callback(self, msg):
        self.curr_pos = msg.pose.pose.position
        
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.curr_yaw = yaw
    
    def arm_and_offboard(self):
        """处理模式切换和解锁，限制请求频率"""
        # 切换OFFBOARD模式（每5秒尝试一次）
        if not self.offboard and (rospy.Time.now() - self.last_req) > rospy.Duration(2.0):
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = "OFFBOARD"
            if self.set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD模式已切换")
            self.last_req = rospy.Time.now()
        
        # 解锁（每5秒尝试一次）
        if self.offboard and not self.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(1.0):
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            if self.arm_client.call(arm_cmd).success:
                rospy.loginfo("无人机已解锁")
            self.last_req = rospy.Time.now()
    
    def check_takeoff_complete(self):
        """检查是否到达起飞位置"""
        dx = abs(self.curr_pos.x - 0)
        dy = abs(self.curr_pos.y - 0)
        dz = abs(self.curr_pos.z - self.takeoff_height)
        return (dx < self.takeoff_threshold) and (dy < self.takeoff_threshold) and (dz < self.takeoff_threshold)
    
    def timer_callback(self, event):
        if self.state == 0:
            # 等待初始化前的准备工作完成（如初始指令发送）
            self.state = 1
            rospy.loginfo("进入初始化阶段：切换OFFBOARD模式并解锁...")
        
        elif self.state == 1:
            # 执行模式切换和解锁
            self.arm_and_offboard()
            if self.armed and self.offboard:
                rospy.loginfo("初始化完成，进入起飞阶段")
                self.state = 2
                self.start_time = rospy.Time.now()
        
        elif self.state == 2:
            # 发布起飞指令
            takeoff_cmd = PositionCommand()
            takeoff_cmd.header.stamp = rospy.Time.now()
            takeoff_cmd.header.frame_id = 'world'
            takeoff_cmd.position = Point(0, 0, self.takeoff_height)
            takeoff_cmd.velocity = Vector3(0, 0, 0.0)  # 缓慢上升
            takeoff_cmd.acceleration = Vector3(0, 0, 0)
            takeoff_cmd.yaw = self.curr_yaw
            takeoff_cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            self.pub_cmd.publish(takeoff_cmd)
            
            # 检查起飞是否完成
            if self.check_takeoff_complete():
                if not self.takeoff_complete:
                    rospy.loginfo("已到达起飞位置，准备进入圆形轨迹")
                    self.takeoff_complete = True
                    self.start_time = rospy.Time.now()
                    self.state = 3
        
        elif self.state == 3:
            # 圆形轨迹逻辑
            t = (rospy.Time.now() - self.start_time).to_sec()
            
            # 平滑过渡到目标半径和速度
            if self.current_radius < self.radius:
                self.current_radius += self.radius / self.step
                self.current_velocity += self.velocity / self.step
                self.current_radius = min(self.current_radius, self.radius)
                self.current_velocity = min(self.current_velocity, self.velocity)
            
            # 计算角度
            if self.current_radius > 0.01:
                self.theta += (self.current_velocity / self.current_radius) * self.dt
            
            # 位置计算
            x = self.current_radius * np.cos(self.theta)
            y = self.current_radius * np.sin(self.theta)
            z = self.takeoff_height + self.z_amplitude * np.sin(2 * np.pi * self.z_frequency * t + self.z_phase)
            
            # 速度计算
            vx = -self.current_velocity * np.sin(self.theta) if self.current_radius > 0.01 else 0.0
            vy = self.current_velocity * np.cos(self.theta) if self.current_radius > 0.01 else 0.0
            vz = 2 * np.pi * self.z_frequency * self.z_amplitude * np.cos(2 * np.pi * self.z_frequency * t + self.z_phase)
            
            # 加速度计算
            ax = - (self.current_velocity**2 / self.current_radius) * np.cos(self.theta) if self.current_radius > 0.01 else 0.0
            ay = - (self.current_velocity**2 / self.current_radius) * np.sin(self.theta) if self.current_radius > 0.01 else 0.0
            az = - (2 * np.pi * self.z_frequency)**2 * self.z_amplitude * np.sin(2 * np.pi * self.z_frequency * t + self.z_phase)
            
            # Yaw计算
            yaw = self.theta + np.pi/2 if self.yaw_control else 0.0
            yaw = normalize_yaw(yaw)
            
            # 发布指令和轨迹
            self.publish_position_command(x, y, z, vx, vy, vz, ax, ay, az, yaw)
            self.publish_reference_trajectory(x, y, z)
    
    def publish_position_command(self, x, y, z, vx, vy, vz, ax, ay, az, yaw):
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.position = Point(x, y, z)
        msg.velocity = Vector3(vx, vy, vz)
        msg.acceleration = Vector3(ax, ay, az)
        msg.yaw = yaw
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