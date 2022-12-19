 #!/usr/bin/env python

import board
import busio
import time
import adafruit_pca9685
from adafruit_motor import servo
import sys
import subprocess

MIN_PULSE_SERVO = 500
MAX_PULSE_SERVO = 2500
MIN_PULSE_THRUS = 1100
MAX_PULSE_THRUS = 1900

# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL


i2c_bus1=(busio.I2C(board.SCL, board.SDA, frequency=1000000)) #i2c port config
pca = adafruit_pca9685.PCA9685(i2c_bus1)  #config pca on selected i2c
pca.frequency = 50  ##frequency is based on actuator data sheet.
thruster1=servo.Servo(pca.channels[0],min_pulse = MIN_PULSE_THRUS,max_pulse = MAX_PULSE_THRUS)
thruster2=servo.Servo(pca.channels[1],min_pulse = MIN_PULSE_THRUS,max_pulse = MAX_PULSE_THRUS)
bowthruster=servo.Servo(pca.channels[2],min_pulse = MIN_PULSE_THRUS,max_pulse = MAX_PULSE_THRUS)
servo_l=servo.Servo(pca.channels[5],min_pulse = MIN_PULSE_SERVO,max_pulse = MAX_PULSE_SERVO)
servo_r=servo.Servo(pca.channels[6],min_pulse = MIN_PULSE_SERVO,max_pulse = MAX_PULSE_SERVO)
water_gun=servo.Servo(pca.channels[15],min_pulse = MIN_PULSE_SERVO,max_pulse = MAX_PULSE_SERVO)


#servo_l.angle = 110
#time.sleep(1)
#servo_l.angle = 50
#time.sleep(1)
#thruster1.angle = 80
#time.sleep(1)
#servo_l.angle = 130
#time.sleep(1)
#thruster1.angle = 90

#servo_r.angle = 120
#time.sleep(1)
#thruster2.angle = 110
#time.sleep(1)
#thruster2.angle = 90

#bowthruster.angle = 110
#time.sleep(1)
#bowthruster.angle = 90

#servo_l.angle = 90
#servo_r.angle = 90

water_gun.angle = 0
time.sleep(3)


#servo_l.angle = 110
#time.sleep(3)
#servo_l.angle = 90
#time.sleep(1)
#servo_l.angle = 70
#time.sleep(3)
#thruster1.angle = 80
#time.sleep(3)
#thruster1.angle = 90
#time.sleep(1)
#thruster1.angle = 130
#time.sleep(2)
#thruster1.angle = 90
#time.sleep(1)
#servo_l.angle = 90
#thruster1.angle = 90
#thruster2.angle = 90
#bowthruster.angle = 90
#time.sleep(3)
