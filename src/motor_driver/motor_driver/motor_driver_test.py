import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
from motor_driver.velocity_smoother import VelocitySmoother

class MotorDriverTest(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.left_ticks = 0
        self.right_ticks = 0
        self.WHEEL_DISTANCE = 0.126  
        self.WHEEL_RADIUS = 0.024
        self.vx = 0
        self.wz = 0
        self.target_vx = 0
        self.target_wz = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.TPR = 241
        self.control_hz = 20.0
        self.get_logger().info("velocity subscriber started.")
        self.smoother = VelocitySmoother()
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_timer = self.create_timer(0.1, self.update_odom)
        self.control_timer = self.create_timer(1 / self.control_hz, self.control_callback)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.simulate_timer = self.create_timer(0.1, self.simultae_ticks)
        self.simulate_dt = 0.1
        
    def cmd_callback(self, msg):
        self.target_vx = msg.linear.x
        self.target_wz = msg.angular.z
        print(f"target velocity{self.target_vx}, {self.target_wz})")

    def control_callback(self):
        self.vx, self.wz = self.smoother.update(self.vx, self.wz, self.target_vx, self.target_wz, 1.0 / self.control_hz)
        # print(f"output velocity{self.vx}, {self.wz})")

    def destroy_node(self):
        self.get_logger().info("Stopping motor driver...")
        super().destroy_node()

    def update_odom(self):
        left_ticks, right_ticks = self.left_ticks, self.right_ticks
        delta_L = left_ticks - self.last_left_ticks
        delta_R = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        dtheta_L = (2 * math.pi * delta_L) / self.TPR
        dtheta_R = (2 * math.pi * delta_R) / self.TPR
        ds_L = self.WHEEL_RADIUS * dtheta_L
        ds_R = self.WHEEL_RADIUS * dtheta_R
        ds = (ds_R + ds_L) / 2.0
        dtheta = (ds_R - ds_L) / self.WHEEL_DISTANCE
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        frame_id = 'odom'
        child_frame_id = 'base_link'
        odom_msg.header.frame_id = frame_id
        odom_msg.child_frame_id = child_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom_msg)
        transform = Transform()
        transform.translation.x = self.x
        transform.translation.y = self.y
        transform.translation.z = 0.0
        transform.rotation = odom_msg.pose.pose.orientation
        tf = TransformStamped()
        tf.header.frame_id = frame_id
        tf.child_frame_id = child_frame_id
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform = transform
        self.tf_broadcaster_.sendTransform(tf)
        # self.get_logger().info(f'Published transform from {frame_id} to {child_frame_id}')

    def simultae_ticks(self):
        v_r = self.vx + (self.wz * self.WHEEL_DISTANCE / 2.0)
        v_l = self.vx - (self.wz * self.WHEEL_DISTANCE / 2.0)

        dtheta_r = (v_r * self.simulate_dt) / self.WHEEL_RADIUS
        dtheta_l = (v_l * self.simulate_dt) / self.WHEEL_RADIUS

        delta_ticks_r = (dtheta_r / (2 * math.pi)) * self.TPR
        delta_ticks_l = (dtheta_l / (2 * math.pi)) * self.TPR

        self.right_ticks += delta_ticks_r
        self.left_ticks += delta_ticks_l
        print(f"left_ticks: {self.left_ticks}, right_ticks: {self.right_ticks}")
def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverTest("motor_driver_test")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
