 #!/usr/bin/env python

# import sys
# print (sys.executable)
# SDA = pin.SDA_1
# SCL = pin.SCL_1
# SDA_1 = pin.SDA
# SCL_1 = pin.SCL

import board
import busio
import time
import adafruit_pca9685
from adafruit_motor import servo

# def motor_initialise():
print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1)) #i2c port config
# i2c_bus1=(busio.I2C(board.SCL, board.SDA)) #i2c port config
print("Initializing ServoKit")
pca = adafruit_pca9685.PCA9685(i2c_bus0)
pca.frequency = 50
print("Initializing ESC")
thruster1=servo.Servo(pca.channels[0],min_pulse = 1100,max_pulse = 1900)
thruster2=servo.Servo(pca.channels[1],min_pulse = 1100,max_pulse = 1900)
bowthruster=servo.Servo(pca.channels[2],min_pulse = 1100,max_pulse = 1900)
servo_l=servo.Servo(pca.channels[5],min_pulse = 700,max_pulse = 2300)
servo_r=servo.Servo(pca.channels[6],min_pulse = 700,max_pulse = 2300)
time.sleep(3)
for i in range(70, 120, 10):
    print(f"Trying angle {i}")
    thruster1.angle=i
    time.sleep(2)
for i in range(70, 120, 10):
    print(f"Trying angle {i}")
    thruster2.angle=i
    time.sleep(2)
for i in range(70, 120, 10):
    print(f"Trying angle {i}")
    bowthruster.angle=i
    time.sleep(2)
thruster1.angle=100
thruster2.angle=100
bowthruster.angle=100
time.sleep(5)
pca.deinit()

quit()
