import time
import math

class PID:
    def __init__(self, kp, ki, kd, output_limit=5.0, integral_limit=5.0,
                 deriv_alpha=0.9, max_delta=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.deriv_alpha = deriv_alpha
        self.max_delta = max_delta      # ⭐输出变化率限制

        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.prev_output = 0.0          # ⭐记录上次输出

        self.D_filtered = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.prev_output = 0.0

    def update(self, target, current):
        now = time.perf_counter()

        if self.prev_time is None:
            # 第一次调用
            self.prev_time = now
            self.prev_error = target - current
            return 0.0

        dt = now - self.prev_time
        self.prev_time = now
        dt = max(dt, 1e-4)

        error = target - current

        # ---- PID 计算 ----
        P = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral, self.integral_limit))
        I = self.ki * self.integral

        derivative = (error - self.prev_error) / dt
        self.D_filtered = self.deriv_alpha * self.D_filtered + \
                          (1 - self.deriv_alpha) * derivative
        D = self.kd * self.D_filtered

        self.prev_error = error

        output = P + I + D

        # ---- 限幅 ----
        output = max(-self.output_limit, min(self.output_limit, output))

        # ---- ⭐输出变化率限制（关键） ----
        delta = output - self.prev_output

        if abs(delta) > self.max_delta:
            output = self.prev_output + math.copysign(self.max_delta, delta)

        self.prev_output = output

        print(f"[debug] P={P:.3f}, I={I:.3f}, D={D:.3f}, out={output:.3f}, dt={dt:.5f}")

        return output


def main():
    pid = PID(kp=1.0, ki=0.1, kd=0.01, output_limit=5.0)
    value = 0.0
    target = 10.0

    while True:
        u = pid.update(target, value)

        # 模拟系统（简单 1 阶惯性系统，而不是 value += u）
        value += u * 0.1    # 模拟慢系统（0.05 可调节系统惯性）

        print(f"value={value:.3f}")

        if abs(value - target) < 0.01:
            break

        time.sleep(0.01)

