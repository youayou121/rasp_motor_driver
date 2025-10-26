import RPi.GPIO as GPIO
import time


left_spin_count = 0
right_spin_count = 0
AL = 16
BL = 20
AR = 26
BR = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(BL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(AL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(AR, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def my_callback_left(channel):
    global left_spin_count
    if GPIO.input(AL):
        if not GPIO.input(BL):
            left_spin_count -= 1
        elif GPIO.input(BL):
            left_spin_count += 1
    print('left: ', left_spin_count)

def my_callback_right(channel):
    global right_spin_count
    if GPIO.input(AR):
        if not GPIO.input(BR):
            right_spin_count += 1
        elif GPIO.input(BR):
            right_spin_count -= 1
    print('right: ', right_spin_count)

GPIO.add_event_detect(AL, GPIO.RISING, callback=my_callback_left)
GPIO.add_event_detect(AR, GPIO.RISING, callback=my_callback_right)
while True:
    print("Runing")
    time.sleep(1)