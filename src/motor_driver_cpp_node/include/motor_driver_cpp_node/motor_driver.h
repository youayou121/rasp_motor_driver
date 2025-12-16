#pragma once
#include <pigpio.h>
#include <atomic>
#include <cmath>
#include <iostream>
#include <cstdint>

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

// Quadrature decoder state
std::atomic<int> left_state(0);
std::atomic<int> right_state(0);
std::atomic<uint32_t> left_last_tick(0);
std::atomic<uint32_t> right_last_tick(0);

// transition table: prev*4 + curr => delta
static const int8_t trans_table[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
};

void encoder_callback(int gpio, int level, uint32_t tick)
{
    if (level == PI_TIMEOUT) return; // ignore timeout events
    const uint32_t DEBOUNCE_US = 50; // microsecond debounce threshold

    if (gpio == E1A || gpio == E1B)
    {
        uint32_t last = right_last_tick.load();
        if (last && (uint32_t)(tick - last) < DEBOUNCE_US) return;
        right_last_tick.store(tick);

        int prev = right_state.load();
        int a = gpioRead(E1A) ? 1 : 0;
        int b = gpioRead(E1B) ? 1 : 0;
        int curr = (a << 1) | b;
        right_state.store(curr);

        int8_t delta = trans_table[prev * 4 + curr];
        if (delta) right_ticks -= delta;
    }
    else if (gpio == E2A || gpio == E2B)
    {
        uint32_t last = left_last_tick.load();
        if (last && (uint32_t)(tick - last) < DEBOUNCE_US) return;
        left_last_tick.store(tick);

        int prev = left_state.load();
        int a = gpioRead(E2A) ? 1 : 0;
        int b = gpioRead(E2B) ? 1 : 0;
        int curr = (a << 1) | b;
        left_state.store(curr);

        int8_t delta = trans_table[prev * 4 + curr];
        if (delta) left_ticks += delta;
    }
}

class MotorDriver
{
public:
    double MAX_SPEED = 1.0;
    int min_duty = 10;

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
        gpioSetAlertFunc(E2B, encoder_callback);
        gpioSetAlertFunc(E1A, encoder_callback);
        gpioSetAlertFunc(E1B, encoder_callback);
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
