#pragma once
#include <algorithm>
#include <cmath>
class VelocitySmoother
{
public:
    explicit VelocitySmoother(double a_vx = 1.0, double a_wz = 1.0) : a_vx_(a_vx), a_wz_(a_wz)
    {
    }

    void update(double current_vx, double current_wz, double target_vx, double target_wz, double dt, double &out_vx, double &out_wz)
    {
        if (dt <= 1e-6)
        {
            out_vx = current_vx;
            out_wz = current_wz;
            return;
        }

        double delta_vx = target_vx - current_vx;
        double max_acc_vx = dt * a_vx_;

        if (delta_vx > 0)
        {
            out_vx = clip(target_vx, current_vx - max_acc_vx, current_vx + max_acc_vx);
        }
        else
        {
            out_vx = clip(target_vx, current_vx + max_acc_vx, current_vx - max_acc_vx);
        }

        double delta_wz = target_wz - current_wz;
        double max_acc_wz = dt * a_wz_;

        if (delta_wz > 0)
        {
            out_wz = clip(target_wz, current_wz - max_acc_wz, current_wz + max_acc_wz);
        }
        else
        {
            out_wz = clip(target_wz, current_wz + max_acc_wz, current_wz - max_acc_wz);
        }
    }

private:
    double a_vx_;
    double a_wz_;

    double clip(double x, double min, double max)
    {
        if (min > max)
        {
            std::swap(min, max);
        }
        if (x > max)
            return max;
        if (x < min)
            return min;
        return x;
    }
};