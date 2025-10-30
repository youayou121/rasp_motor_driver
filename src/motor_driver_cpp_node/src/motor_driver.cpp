#include <wiringPi.h>
#include <iostream>
#include <cmath>

class MotorDriver {
public:
    // GPIO 引脚定义（BCM编号）
    const int PWMA = 18;
    const int AIN1 = 14;
    const int AIN2 = 15;
    const int PWMB = 19;
    const int BIN1 = 23;
    const int BIN2 = 24;

    const int AL = 16;
    const int BL = 20;
    const int AR = 26;
    const int BR = 21;

    const double MAX_SPEED = 1.0;

    volatile long left_ticks = 0;
    volatile long right_ticks = 0;

    MotorDriver() {
        wiringPiSetupGpio(); // 使用 BCM 编号

        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        pinMode(PWMA, PWM_OUTPUT);
        pinMode(PWMB, PWM_OUTPUT);

        // 初始化 PWM
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(1024);
        pwmSetClock(32);

        // 编码器输入引脚
        pinMode(AL, INPUT);
        pinMode(BL, INPUT);
        pinMode(AR, INPUT);
        pinMode(BR, INPUT);

        pullUpDnControl(AL, PUD_UP);
        pullUpDnControl(BL, PUD_UP);
        pullUpDnControl(AR, PUD_UP);
        pullUpDnControl(BR, PUD_UP);

        // 注册中断回调
        wiringPiISR(AL, INT_EDGE_RISING, &MotorDriver::left_callback_wrapper);
        wiringPiISR(AR, INT_EDGE_RISING, &MotorDriver::right_callback_wrapper);

        instance = this;
    }

    ~MotorDriver() {
        stop();
    }

    // 设置左电机速度（单位：m/s）
    void set_left_motor(double velocity) {
        double duty = std::min(std::abs(velocity) / MAX_SPEED, 1.0);
        int pwm_value = (int)(duty * 1023);

        if (velocity > 0) {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
        } else if (velocity < 0) {
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
        } else {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, LOW);
        }
        pwmWrite(PWMB, pwm_value);
    }

    // 设置右电机速度
    void set_right_motor(double velocity) {
        double duty = std::min(std::abs(velocity) / MAX_SPEED, 1.0);
        int pwm_value = (int)(duty * 1023);

        if (velocity > 0) {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
        } else if (velocity < 0) {
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
        } else {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);
        }
        pwmWrite(PWMA, pwm_value);
    }

    // 停止所有电机
    void stop() {
        pwmWrite(PWMA, 0);
        pwmWrite(PWMB, 0);
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }

    // 获取编码器计数
    void get_ticks(long &left, long &right) {
        left = left_ticks;
        right = right_ticks;
    }

private:
    // 静态回调函数包装（wiringPiISR 不支持成员函数）
    static void left_callback_wrapper() {
        if (instance) instance->callback_left();
    }
    static void right_callback_wrapper() {
        if (instance) instance->callback_right();
    }

    void callback_left() {
        if (digitalRead(AL)) {
            if (!digitalRead(BL))
                left_ticks--;
            else
                left_ticks++;
        }
    }

    void callback_right() {
        if (digitalRead(AR)) {
            if (!digitalRead(BR))
                right_ticks++;
            else
                right_ticks--;
        }
    }

    static MotorDriver* instance;
};