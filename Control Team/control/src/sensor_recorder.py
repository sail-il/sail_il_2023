#!/usr/bin/env python

# compass imports
from tkinter import HORIZONTAL
import gy271compass as GY271
import math
#############################

#imu imports
from pdb import line_prefix
from re import A
import time
import board
import busio
import adafruit_mpu6050
import math
#######################

# csv imports
import csv
################

#csv configuration
f = open('drift.csv', 'w' , newline="")  #giving a name to the csv file
# IMPORTANT!!
# if the same file name is applied every time
# the new file will run over the older file

writer = csv.writer(f)
headers = ['time', 'angular velocity' , 'v0x' , 'v0y' , 'v0z' , 'aX' , 'aY' , 'aZ']
writer.writerow(headers)
#################################################

#imu configuration
i2c = busio.I2C(board.SCL_1, board.SDA_1)  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c, 0x68)
#########################################3

#compass configuration
sensor = GY271.compass(address=0x0d)
######################

#frequency configuration
frequency = 10  #10 hz
delta_time = 1.0 / frequency
###########################

# initial configuration
last_angle=0
v0x=0
v0y=0
v0z=0
time_stamp =0
############################

print('finished initialization')

while (time_stamp<= 60):

    # angular velocity
    angle = sensor.get_bearing()
    radian_angle = (angle * math.pi) / 180 # radians conversion 
    angular_velocity = (radian_angle - last_angle)/delta_time 
    last_angle = radian_angle
    ###################################################

    # 3 axis velocity
    accelerationX = mpu.acceleration[0]
    accelerationY = mpu.acceleration[1]
    accelerationZ = mpu.acceleration[2]

    v0x = v0x + (accelerationX * delta_time)
    v0y = v0y + (accelerationY * delta_time)
    v0z = v0z + (accelerationZ * delta_time)
    ############################################
    
    time_stamp += delta_time
    print([time_stamp , v0x , v0y , accelerationX ,accelerationY])
    writer.writerow([time_stamp , angular_velocity , v0x , v0y , v0z , accelerationX ,accelerationY ,accelerationZ])     
    time.sleep(delta_time)


f.close()
print('finished recording')