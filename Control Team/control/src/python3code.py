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
import sys
import subprocess

MIN_PULSE_SERVO = 500
MAX_PULSE_SERVO = 2500
MIN_PULSE_THRUS = 1100
MAX_PULSE_THRUS = 1900

# def motor_initialise():
#i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1)) #i2c port config
i2c_bus1=(busio.I2C(board.SCL, board.SDA, frequency=1000000)) #i2c port config
pca = adafruit_pca9685.PCA9685(i2c_bus1)
pca.frequency = 50
thruster1=servo.Servo(pca.channels[0],min_pulse = MIN_PULSE_THRUS,max_pulse = MAX_PULSE_THRUS)
thruster2=servo.Servo(pca.channels[1],min_pulse = MIN_PULSE_THRUS,max_pulse = MAX_PULSE_THRUS)
bowthruster=servo.Servo(pca.channels[2],min_pulse = MIN_PULSE_THRUS,max_pulse = MAX_PULSE_THRUS)
servo_l=servo.Servo(pca.channels[5],min_pulse = MIN_PULSE_SERVO,max_pulse = MAX_PULSE_SERVO)
servo_r=servo.Servo(pca.channels[6],min_pulse = MIN_PULSE_SERVO,max_pulse = MAX_PULSE_SERVO)
cannon_chanel=pca.channels[8]

def callback(motor_commands):
    thruster1.angle=motor_commands[0] #calculation to convert throttle value to pwm signal based on servo motor
    thruster2.angle=motor_commands[1]
    bowthruster.angle=motor_commands[2]
    servo_r.angle=motor_commands[3]
    servo_l.angle=motor_commands[4]
    if (motor_commands[5]==5): #trun on cannon
        cannon_chanel.duty_cycle=0xffff
       # pca.pwm.setPWM(cannon_chanel,4096,0)
    elif (motor_commands[5]==6):  #trun off cannon
        #pca.pwm.setPWM(cannon_chanel,0,4096)
        cannon_chanel.duty_cycle=0x0000
        
    
    

if __name__ == '__main__':
    for line in sys.stdin:
        line = line.split(" ")
        callback([
            float(line[0]),
            float(line[1]),
            float(line[2]),
            float(line[3]),
            float(line[4]),
            float(line[5])])