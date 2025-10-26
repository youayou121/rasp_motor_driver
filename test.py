#motorTest.py
#导入 GPIO库
import RPi.GPIO as GPIO
import time

#设置 GPIO 模式为 BCM
GPIO.setmode(GPIO.BCM)

#定义引脚
STBY = 27
PWMA = 18
AIN1 = 14
AIN2 = 15
PWMB = 19
BIN1 = 23
BIN2 = 24

#设置 GPIO 的工作方式
GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
pwma = GPIO.PWM(PWMA,300)
pwmb = GPIO.PWM(PWMB,300)


# 前进或后退（大于零前进，小于零后退）
def goForward(speed):
    if(speed>=0):
        GPIO.output(AIN1,GPIO.LOW)
        GPIO.output(AIN2,GPIO.HIGH)
        GPIO.output(BIN1,GPIO.LOW)
        GPIO.output(BIN2,GPIO.HIGH)
        pwma.start(speed)
        pwmb.start(speed)
        time.sleep(0.02)
    else:
        GPIO.output(AIN2,GPIO.LOW)
        GPIO.output(AIN1,GPIO.HIGH)
        GPIO.output(BIN2,GPIO.LOW)
        GPIO.output(BIN1,GPIO.HIGH)
        pwma.start(-speed)
        pwmb.start(-speed)
        time.sleep(0.02)

# 左转或右转（大于零左转，小于零右转）
def turnLeft(speed):
    if(speed>=0):
        GPIO.output(AIN2,GPIO.LOW)
        GPIO.output(AIN1,GPIO.HIGH)
        GPIO.output(BIN1,GPIO.LOW)
        GPIO.output(BIN2,GPIO.HIGH)
        pwma.start(speed)
        pwmb.start(speed)
        time.sleep(0.02)
    else:
        GPIO.output(AIN1,GPIO.LOW)
        GPIO.output(AIN2,GPIO.HIGH)
        GPIO.output(BIN2,GPIO.LOW)
        GPIO.output(BIN1,GPIO.HIGH)
        pwma.start(-speed)
        pwmb.start(-speed)
        time.sleep(0.02)

def motorStop():
    GPIO.output(AIN1,GPIO.LOW)
    GPIO.output(AIN2,GPIO.LOW)

GPIO.output(STBY,GPIO.HIGH)
#以60%的速度前进
goForward(10)
time.sleep(2)
#以60%的速度后退
goForward(-10)
time.sleep(2)
#左转
turnLeft(10)
time.sleep(2)
#右转
turnLeft(-10)
time.sleep(2)
#停止
motorStop()

pwma.stop()
pwmb.stop()
GPIO.cleanup()