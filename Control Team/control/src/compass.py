from tkinter import HORIZONTAL
import gy271compass as GY271
sensor = GY271.compass(address=0x0d)
from time import sleep
import math

frequency = 2  #20 hz
delta_time = 1.0 / frequency
last_angle=0

print('[Press CTRL + C to end the script!]')
try:
    while True:
        angle = sensor.get_bearing()
        radian_angle = (angle * math.pi) / 180 # radians conversion 
        angular_velocity = (radian_angle - last_angle)/delta_time 
        last_angle = radian_angle
              
        print(f"angular velocity is: {angular_velocity} radians per second")
        print('Heading Angle = {}°'.format(angle))
        
        sleep(delta_time)

except KeyboardInterrupt:
    print('\nScript end!')