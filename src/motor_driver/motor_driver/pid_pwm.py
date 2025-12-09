import threading
import time
import RPi.GPIO as GPIO
class PIDPWM(threading.Thread):
    def __init__(self, pwm, kp = 1.0, ki = 0.1, kd = 0.01, output_limit = 1.0, integral_limit = 10.0, deriv_alpha = 0.9, max_delta = 0.5):
        super().__init__()
        self.pwm = pwm
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.deriv_alpha = deriv_alpha
        self.max_delta = max_delta

        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.prev_output = 0.0
        self.D_filtered = 0.0

        self.target_duty = 0.0
        self.current_duty = 0.0  
        self.loop_dt = 0.01
        self.running = True

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.prev_output = 0.0

    def update(self, target, current):
        now = time.perf_counter()

        if self.prev_time is None:
            self.prev_time = now
            self.prev_error = target - current
            return 0.0

        dt = now - self.prev_time
        self.prev_time = now
        dt = max(dt, self.loop_dt)

        error = target - current

        P = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        I = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        self.D_filtered = self.deriv_alpha * self.D_filtered + (1 - self.deriv_alpha) * derivative
        D = self.kd * self.D_filtered

        self.prev_error = error

        output = P + I + D

        output = max(-self.output_limit, min(self.output_limit, output))

        delta = output - self.prev_output

        if abs(delta) > self.max_delta:
            output = self.prev_output + math.copysign(self.max_delta, delta)

        self.prev_output = output

        print(f"[debug] P={P:.3f}, I={I:.3f}, D={D:.3f}, out={output:.3f}, dt={dt:.5f}")

        return output
    
    def run(self):
        while self.running:
            output = self.update(self.target_duty, self.current_duty)
            self.current_duty += output * self.loop_dt
            self.current_duty = max(0, min(100, self.current_duty))
            self.pwm.ChangeDutyCycle(self.current_duty)
            time.sleep(self.loop_dt)
    
    def set_target(self, duty):
        if((self.target_duty - self.current_duty) * (duty - self.current_duty) < 0):
            self.reset()
        self.target_duty = duty
    def stop(self):
        self.running = False
        self.pwm.stop()