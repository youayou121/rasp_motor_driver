
class VelocitySmoother:
    def __init__(self, a_vx = 1.0, a_wz = 1.0):
        self.a_vx = a_vx
        self.a_wz = a_wz

    def update(self, current_vx, current_wz, target_vx, target_wz, dt):
        if dt <= 1e-6:
            return current_vx, current_wz
        else:
            delta_vx = target_vx - current_vx
            max_acc_vx = dt * self.a_vx
            if (delta_vx > 0):
                current_vx = self.clip(target_vx, current_vx - max_acc_vx, current_vx + max_acc_vx)
            else:
                current_vx = self.clip(target_vx, current_vx + max_acc_vx, current_vx - max_acc_vx)
            delta_wz = target_wz - current_wz
            max_acc_wz = dt * self.a_wz
            if (delta_wz > 0):
                current_wz = self.clip(target_wz, current_wz - max_acc_wz, current_wz + max_acc_wz)
            else:
                current_wz = self.clip(target_wz, current_wz + max_acc_wz, current_wz - max_acc_wz)

        return current_vx, current_wz
    def clip(self, x, min, max):
        if(min > max):
            temp = min
            min = max
            max = temp
        if (x > max):
            x = max

        if (x < min):
            x = min
        return x
