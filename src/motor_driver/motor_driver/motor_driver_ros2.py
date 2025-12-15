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
from motor_driver.motor_driver_pigpio import MotorDriver

class MotorDriverROS(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.WHEEL_DISTANCE = 0.126  
        self.WHEEL_RADIUS = 0.024
        self.vx = 0
        self.wz = 0
        self.target_vx = 0
        self.target_wz = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.TPR_L = 260
        self.TPR_R = 260
        self.control_hz = 20.0
        self.motor_driver = MotorDriver()
        self.get_logger().info("velocity subscriber started.")
        self.smoother = VelocitySmoother()
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_timer = self.create_timer(0.5, self.update_odom)
        self.control_timer = self.create_timer(1 / self.control_hz, self.control_callback)
        self.tf_broadcaster_ = TransformBroadcaster(self)
    def cmd_callback(self, msg):
        self.target_vx = msg.linear.x
        self.target_wz = msg.angular.z
        # print(f"target velocity{self.target_vx}, {self.target_wz})")

    def control_callback(self):
        self.vx, self.wz = self.smoother.update(self.vx, self.wz, self.target_vx, self.target_wz, 1.0 / self.control_hz)
        # print(f"output velocity{self.vx}, {self.wz})")
        v_l = self.vx - (self.wz * self.WHEEL_DISTANCE / 2.0)
        v_r = self.vx + (self.wz * self.WHEEL_DISTANCE / 2.0)
        # print(f"v_l: {v_l}, v_r: {v_r}")
        duty_l = self.motor_driver.set_left_motor(v_l)
        duty_r = self.motor_driver.set_right_motor(v_r)
        # print(f"duty: {duty_l, duty_r}")

    def destroy_node(self):
        self.get_logger().info("Stopping motor driver...")
        self.motor_driver.stop()
        super().destroy_node()

    def update_odom(self):
        left_ticks, right_ticks = self.motor_driver.get_ticks()
        print(f"last ticks {self.last_left_ticks, self.last_right_ticks}")
        print(f"ticks: {left_ticks, right_ticks}")
        delta_L = left_ticks - self.last_left_ticks
        delta_R = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        ds_L = 2 * math.pi * self.WHEEL_RADIUS * delta_L / self.TPR_L
        ds_R = 2 * math.pi * self.WHEEL_RADIUS * delta_R / self.TPR_R
        ds = (ds_R + ds_L) / 2.0
        dtheta = (ds_R - ds_L) / self.WHEEL_DISTANCE
        print(f"ds: {ds}, dtheta: {dtheta}")
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        if self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        elif self.theta < -math.pi:
            self.theta += 2.0 * math.pi
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

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverROS("motor_driver_ros")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
