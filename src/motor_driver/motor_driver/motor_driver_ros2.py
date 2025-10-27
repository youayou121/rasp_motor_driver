import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
# from motor_driver.motor_driver import MotorDriver


class MotorDriverROS(Node):
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
        self.TPR = 260
        # self.motor_driver = MotorDriver()
        self.get_logger().info("velocity subscriber started.")
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_timer = self.create_timer(0.1, self.update_odom)
        self.simulate_timer = self.create_timer(0.1, self.simultae_ticks)

    def cmd_callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z
        self.vx = vx
        self.wz = wz
        print(f"get vel input({vx}, {wz})")
        v_l = vx - (wz * self.WHEEL_DISTANCE / 2.0)
        v_r = vx + (wz * self.WHEEL_DISTANCE / 2.0)
        print(f"v_l: {v_l}, v_r: {v_r}")
        # duty_l = self.motor_driver.set_left_motor(v_l)
        # duty_r = self.motor_driver.set_right_motor(v_r)
        # print(f"l_duty: {duty_l}, r_duty: {duty_r}\n")

    def destroy_node(self):
        self.get_logger().info("Stopping motor driver...")
        # self.motor_driver.stop()
        super().destroy_node()

    def update_odom(self):
        # left_ticks, right_ticks = self.motor_driver.get_ticks()
        left_ticks, right_ticks = self.left_ticks, self.right_ticks
        delta_L = left_ticks - self.last_left_ticks
        delta_R = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        dtheta_L = (2 * math.pi * delta_L) / self.TPR
        dtheta_R = (2 * math.pi * delta_R) / self.TPR
        ds_L = self.R * dtheta_L
        ds_R = self.R * dtheta_R
        ds = (ds_R + ds_L) / 2.0
        dtheta = (ds_R - ds_L) / self.WHEEL_DISTANCE
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_pub.publish(odom_msg)

    def simultae_ticks():
        v_r = self.vx + (self.wz * self.WHEEL_DISTANCE / 2.0)
        v_l = self.vx - (self.wz * self.WHEEL_DISTANCE / 2.0)

        dtheta_r = (v_r * dt) / self.WHEEL_RADIUS
        dtheta_l = (v_l * dt) / self.WHEEL_RADIUS

        delta_ticks_r = (dtheta_r / (2 * math.pi)) * self.TPR
        delta_ticks_l = (dtheta_l / (2 * math.pi)) * self.TPR

        self.right_ticks += delta_ticks_r
        self.left_ticks += delta_ticks_l

        return delta_ticks_l, delta_ticks_r
def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverROS("motor_driver_ros")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
