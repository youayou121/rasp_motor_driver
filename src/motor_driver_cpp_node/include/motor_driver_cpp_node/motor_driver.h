#pragma once
#include <pigpio.h>
#include <atomic>
#include <cmath>
#include <iostream>

#define PWMA 18
#define AIN1 14
#define AIN2 15

#define PWMB 19
#define BIN1 23
#define BIN2 24

#define E2A 16
#define E2B 20
#define E1A 26
#define E1B 21

std::atomic<int> left_ticks(0);
std::atomic<int> right_ticks(0);

void encoder_callback(int gpio, int level, uint32_t tick)
{
    if (level == PI_HIGH)
    {
        if (gpio == E1A)
        {
            if (gpioRead(E1B))
                right_ticks--;
            else
                right_ticks++;
        }
        else if (gpio == E2A)
        {
            if (gpioRead(E2B))
                left_ticks++;
            else
                left_ticks--;
        }
    }
}

class MotorDriver
{
public:
    double MAX_SPEED = 1.0;
    int min_duty = 4;

    MotorDriver()
    {
        if (gpioInitialise() < 0)
        {
            std::cerr << "pigpio init failed\n";
            std::exit(1);
        }

        gpioSetMode(AIN1, PI_OUTPUT);
        gpioSetMode(AIN2, PI_OUTPUT);
        gpioSetMode(BIN1, PI_OUTPUT);
        gpioSetMode(BIN2, PI_OUTPUT);

        // 编码器 IO
        gpioSetMode(E2A, PI_INPUT);
        gpioSetMode(E2B, PI_INPUT);
        gpioSetMode(E1A, PI_INPUT);
        gpioSetMode(E1B, PI_INPUT);

        gpioSetPullUpDown(E2A, PI_PUD_UP);
        gpioSetPullUpDown(E2B, PI_PUD_UP);
        gpioSetPullUpDown(E1A, PI_PUD_UP);
        gpioSetPullUpDown(E1B, PI_PUD_UP);

        gpioHardwarePWM(PWMA, 1000, 0);
        gpioHardwarePWM(PWMB, 1000, 0);

        gpioSetAlertFunc(E2A, encoder_callback);
        gpioSetAlertFunc(E1A, encoder_callback);
    }
    void set_left_motor(double velocity)
    {
        int duty = calc_duty(velocity);

        if (velocity > 0)
        {
            gpioWrite(BIN1, 0);
            gpioWrite(BIN2, 1);
        }
        else if (velocity < 0)
        {
            gpioWrite(BIN1, 1);
            gpioWrite(BIN2, 0);
        }
        else
        {
            duty = 0;
            gpioWrite(BIN1, 0);
            gpioWrite(BIN2, 0);
        }

        gpioHardwarePWM(PWMB, 1000, duty * 10000);
    }

    void set_right_motor(double velocity)
    {
        int duty = calc_duty(velocity);

        if (velocity > 0)
        {
            gpioWrite(AIN1, 0);
            gpioWrite(AIN2, 1);
        }
        else if (velocity < 0)
        {
            gpioWrite(AIN1, 1);
            gpioWrite(AIN2, 0);
        }
        else
        {
            duty = 0;
            gpioWrite(AIN1, 0);
            gpioWrite(AIN2, 0);
        }

        gpioHardwarePWM(PWMA, 1000, duty * 10000);
    }

    void get_ticks(int &l, int &r)
    {
        l = left_ticks.load();
        r = right_ticks.load();
    }

    void stop()
    {
        gpioHardwarePWM(PWMA, 1000, 0);
        gpioHardwarePWM(PWMB, 1000, 0);
        gpioTerminate();
    }

private:
    int calc_duty(double velocity)
    {
        int duty = std::min(100, (int)(std::abs(velocity) / MAX_SPEED * 100));
        if (duty > 0)
            duty = std::max(min_duty, duty);
        return duty;
    }
};
