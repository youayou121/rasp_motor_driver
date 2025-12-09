import RPi.GPIO as GPIO
import time
from motor_driver.pid_pwm import PIDPWM
class MotorDriver():
    def __init__(self):
        self.PWMA = 18
        self.AIN1 = 14
        self.AIN2 = 15
        self.PWMB = 19
        self.BIN1 = 23
        self.BIN2 = 24
        self.MAX_SPEED = 1.0
        self.min_duty = 5
        self.left_ticks = 0
        self.right_ticks = 0

        self.E2A = 16
        self.E2B = 20
        self.E1A = 26
        self.E1B = 21

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.AIN1, self.AIN2, self.BIN1, self.BIN2], GPIO.OUT)
        GPIO.setup([self.PWMA, self.PWMB], GPIO.OUT)
        self.pwma = GPIO.PWM(self.PWMA, 1000)
        self.pwmb = GPIO.PWM(self.PWMB, 1000)
        self.pwma.start(0)
        self.pwmb.start(0)
        self.pid_pwma = PIDPWM(self.pwma)
        self.pid_pwmb = PIDPWM(self.pwmb)
        self.pid_pwma.start()
        self.pid_pwmb.start()
        GPIO.setup(self.E2B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.E2A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.E1B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.E1A, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.E2A, GPIO.RISING, callback=self.callback_left)
        GPIO.add_event_detect(self.E1A, GPIO.RISING, callback=self.callback_right)

    def set_left_motor(self, velocity):
        duty = 0.0
        if velocity > self.MAX_SPEED:
            duty = 100.0
        else:
            duty = abs(velocity) / self.MAX_SPEED * 100.0
        if abs(velocity) > 0:
            duty = max(self.min_duty, duty)
        else: 
            duty = 0
        if velocity > 0:
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
        elif velocity < 0:
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
        else:
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.LOW)

        self.pid_pwmb.set_target(duty)
        return duty
    
    def set_right_motor(self, velocity):
        duty = 0.0
        if velocity > self.MAX_SPEED:
            duty = 100.0
        else:
            duty = abs(velocity) / self.MAX_SPEED * 100.0
        if abs(velocity) > 0:
            duty = max(self.min_duty, duty)
        else: 
            duty = 0
        if velocity > 0:
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
        elif velocity < 0:
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
        else:
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.LOW)
        self.pid_pwma.set_target(duty)
        return duty
    
    def stop(self):
        self.pid_pwma.stop()
        self.pid_pwmb.stop()
        GPIO.cleanup()

    def get_ticks(self):
        return self.left_ticks, self.right_ticks

    def callback_left(self, channel):
        if GPIO.input(self.E2A):
            if not GPIO.input(self.E2B):
                self.left_ticks -= 1
            elif GPIO.input(self.E2B):
                self.left_ticks += 1

    def callback_right(self, channel):
        if GPIO.input(self.E1A):
            if not GPIO.input(self.E1B):
                self.right_ticks += 1
            elif GPIO.input(self.E1B):
                self.right_ticks -= 1

    