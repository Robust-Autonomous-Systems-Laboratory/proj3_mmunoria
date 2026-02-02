import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np
import math

class DeadReckoningEstimator(Node):
    def __init__(self):
        super().__init__('estimator_node')

        
        self.cmd_x, self.cmd_y, self.cmd_theta = 0.0, 0.0, 0.0
        self.imu_x, self.imu_y, self.imu_theta = 0.0, 0.0, 0.0
        self.imu_vx, self.imu_vy = 0.0, 0.0

        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.bias_samples = 0
        self.bias_N = 100  # ~first 100 IMU msgs

        self.last_cmd_time = None
        self.last_imu_time = None

       
        self.cmd_path = Path()
        self.cmd_path.header.frame_id = "odom"
        self.imu_path = Path()
        self.imu_path.header.frame_id = "odom"

       
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_callback, sub_qos)
        self.create_subscription(Imu, '/imu', self.imu_callback, sub_qos)

        self.cmd_odom_pub = self.create_publisher(Odometry, '/dead_reckoning/odom', odom_qos)
        self.imu_odom_pub = self.create_publisher(Odometry, '/imu_integration/odom',odom_qos)

        self.cmd_path_pub = self.create_publisher(Path, '/dead_reckoning/path', path_qos)
        self.imu_path_pub = self.create_publisher(Path, '/imu_integration/path', path_qos)

    def get_dt(self, msg, last_time):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if last_time is None:
            return 0.0, t
        return t - last_time, t

    def cmd_callback(self, msg):
        
        t_now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_cmd_time is None:
            self.last_cmd_time = t_now
            return
        dt = t_now - self.last_cmd_time
        self.last_cmd_time = t_now

        v = msg.twist.linear.x
        w = msg.twist.angular.z

        # Update State
        self.cmd_theta += w * dt
        self.cmd_x += v * math.cos(self.cmd_theta) * dt
        self.cmd_y += v * math.sin(self.cmd_theta) * dt

        self.publish_odometry(self.cmd_odom_pub, self.cmd_path_pub, self.cmd_path,
                             self.cmd_x, self.cmd_y, self.cmd_theta)

    def imu_callback(self, msg):
        dt, self.last_imu_time = self.get_dt(msg, self.last_imu_time)
        if dt <= 0: return

        # 1. Orientation Integration
        wz = msg.angular_velocity.z
        self.imu_theta += wz * dt


        ax_b = float(msg.linear_acceleration.x)
        ay_b = float(msg.linear_acceleration.y)

        # estimate bias for first N samples
        if self.bias_samples < self.bias_N:
            self.ax_bias += ax_b
            self.ay_bias += ay_b
            self.bias_samples += 1
            if self.bias_samples == self.bias_N:
                self.ax_bias /= self.bias_N
                self.ay_bias /= self.bias_N
            return  # don't integrate while calibrating

        # subtract bias
        ax_b -= self.ax_bias
        ay_b -= self.ay_bias

        # 2. Transform Body Accel to World Accel
        ax_body = msg.linear_acceleration.x
        ay_body = msg.linear_acceleration.y

        ax_world = ax_body * math.cos(self.imu_theta) - ay_body * math.sin(self.imu_theta)
        ay_world = ax_body * math.sin(self.imu_theta) + ay_body * math.cos(self.imu_theta)

        # 3. Double Integration
        self.imu_vx += ax_world * dt
        self.imu_vy += ay_world * dt
        self.imu_x += self.imu_vx * dt
        self.imu_y += self.imu_vy * dt

        self.publish_odometry(self.imu_odom_pub, self.imu_path_pub, self.imu_path,
                             self.imu_x, self.imu_y, self.imu_theta)

    def publish_odometry(self, odom_pub, path_pub, path_obj, x, y, theta):

        # Create Odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom_pub.publish(odom)

        # Create Path
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        path_obj.poses.append(pose)
        path_obj.header.stamp = odom.header.stamp
        path_pub.publish(path_obj)


        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()