import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# ==== TB6612引脚定义 ====

PWMA = 18
AIN1 = 14
AIN2 = 15
PWMB = 19
BIN1 = 23
BIN2 = 24

# 机器人参数
WHEEL_RADIUS = 0.024   # m
WHEEL_BASE = 0.126     # m
MAX_SPEED = 0.05        # m/s (根据你的电机性能调整)

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.get_logger().info("Motor driver node started.")

        # 初始化GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([AIN1, AIN2, BIN1, BIN2], GPIO.OUT)
        GPIO.setup([PWMA, PWMB], GPIO.OUT)

        # PWM频率设为1kHz
        self.pwma = GPIO.PWM(PWMA, 1000)
        self.pwmb = GPIO.PWM(PWMB, 1000)
        self.pwma.start(0)
        self.pwmb.start(0)

        # 订阅cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        # 差速运动学公式
        v_r = v + (omega * WHEEL_BASE / 2.0)
        v_l = v - (omega * WHEEL_BASE / 2.0)

        self.set_motor('left', v_l)
        self.set_motor('right', v_r)
        print("cmd callback")
    def set_motor(self, side, velocity):
        # 计算占空比
        # duty = abs(velocity) / MAX_SPEED * 100.0
        # duty = max(0, min(100, duty))  # 限制范围
        duty = 10
        if side == 'left':
            in1, in2, pwm = BIN1, BIN2, self.pwmb
        else:
            in1, in2, pwm = AIN1, AIN2, self.pwma

        # 控制方向
        if velocity > 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        elif velocity < 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)

        pwm.ChangeDutyCycle(duty)

    def destroy_node(self):
        self.get_logger().info("Stopping motor driver...")
        self.pwma.stop()
        self.pwmb.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
