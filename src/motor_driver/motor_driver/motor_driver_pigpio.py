import pigpio
class MotorDriver:
    def __init__(self):
        self.PWMA = 18
        self.AIN1 = 14
        self.AIN2 = 15
        self.PWMB = 19
        self.BIN1 = 23
        self.BIN2 = 24
        self.MAX_SPEED = 1.0
        self.MAX_DUTY = 100.0
        self.min_duty = 4.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.E2A = 16 
        self.E2B = 20  
        self.E1A = 26  
        self.E1B = 21

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running")

        for pin in [self.AIN1, self.AIN2, self.BIN1, self.BIN2]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
        for pin in [self.E2A, self.E2B, self.E1A, self.E1B]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)

        self.pi.hardware_PWM(self.PWMA, 1000, 0)
        self.pi.hardware_PWM(self.PWMB, 1000, 0)

        # ===== 编码器回调 =====
        self.cb_left = self.pi.callback(self.E2A, pigpio.RISING_EDGE, self.callback_left)
        self.cb_right = self.pi.callback(self.E1A, pigpio.RISING_EDGE, self.callback_right)

    def set_pwm_a(self, duty):
        duty = max(0.0, min(self.MAX_DUTY, duty))
        self.pi.hardware_PWM(self.PWMA, 1000, int(duty * 10000))

    def set_pwm_b(self, duty):
        duty = max(0.0, min(self.MAX_DUTY, duty))
        self.pi.hardware_PWM(self.PWMB, 1000, int(duty * 10000))

    def set_left_motor(self, velocity):
        duty = 0.0
        if abs(velocity) > self.MAX_SPEED:
            duty = 100.0
        else:
            duty = abs(velocity) / self.MAX_SPEED * 100.0

        if velocity > 0:
            duty = max(self.min_duty, duty)
            self.pi.write(self.BIN1, 0)
            self.pi.write(self.BIN2, 1)
        else:
            duty = min(self.min_duty, duty)
            self.pi.write(self.BIN1, 0)
            self.pi.write(self.BIN2, 0)

        self.set_pwm_b(duty)

    def set_right_motor(self, velocity):
        duty = 0.0
        if abs(velocity) > self.MAX_SPEED:
            duty = 100.0
        else:
            duty = abs(velocity) / self.MAX_SPEED * 100.0

        if velocity > 0:
            duty = max(self.min_duty, duty)
            self.pi.write(self.AIN1, 0)
            self.pi.write(self.AIN2, 1)
        else:
            duty = min(self.min_duty, duty)
            self.pi.write(self.AIN1, 0)
            self.pi.write(self.AIN2, 1)

        self.set_pwm_a(duty)

    def callback_left(self, gpio, level, tick):
        if level == 1:
            if self.pi.read(self.E2B):
                self.left_ticks += 1
            else:
                self.left_ticks -= 1

    def callback_right(self, gpio, level, tick):
        if level == 1:
            if self.pi.read(self.E1B):
                self.right_ticks -= 1
            else:
                self.right_ticks += 1

    def get_ticks(self):
        return self.left_ticks, self.right_ticks

    def reset_ticks(self):
        self.left_ticks = 0
        self.right_ticks = 0

    def stop(self):
        self.set_pwm_a(0)
        self.set_pwm_b(0)
        self.cb_left.cancel()
        self.cb_right.cancel()
        self.pi.stop()
