#!/usr/bin/env python
PKG = 'control'
import roslib; roslib.load_manifest(PKG)
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
import subprocess
import math
import numpy
import time
from numpy_true_talker import getkey

straight=90.0
stop_motor=94.0
stop_front_motor=94.0
front_angle=0.0
right_turn=70.0
left_turn=110.0
T=30
.0
K=0.2

servo_proc = subprocess.Popen(["python3.7", "/home/pinksail/control_ws/src/control/src/python3code.py"], stdin=subprocess.PIPE)

#converts the list message to python 3.7 format
#because ros runs on python2 but circuitpython runs on python 3
def callback(data):
    print("Received command: {}".format(data))
    x = float(data.linear.x)
    y = float(data.linear.y)
    z = float(data.angular.z)
    #vectoring data:

    #finding direction angle from arctan. arctan gives values from -90 to 90 degrees. in order to find the true angle between 0 to 360 the below calculations are needed
    try:
        vector_angle_degress = ((math.atan(float(x)/float(y)))/math.pi)*180
        if (float(x) >= 0 and float(y) >= 0):
            true_angle = vector_angle_degress
        elif (float(x) >= 0 and float(y) < 0):
            true_angle = vector_angle_degress + 180
        elif (float(x) < 0 and float(y) >= 0):
            true_angle = vector_angle_degress + 360
        elif (float(x )< 0 and float(y) < 0):
            true_angle = vector_angle_degress + 180
    except ZeroDivisionError: 
        true_angle = 0

#factoring the needed servo angle into angle command between the range of 0 - 180
    if true_angle <= 90:
        servo_angle = 90 - true_angle
        thruster_factor = 1 # the boat cant operate at max reverse speed
    elif true_angle > 90 and true_angle <= 180:
        servo_angle = true_angle - 90
        thruster_factor = 1 # the boat cant operate at max reverse speed
    elif true_angle > 180 and true_angle <= 270:
        servo_angle = 270 - true_angle
        thruster_factor = -0.25    
    else:
        servo_angle = true_angle - 270
        thruster_factor = -0.25
    #the servo command
    servo_angle_command = servo_angle
    #speed vector size:
    speed_vector_size = math.sqrt(((abs(float(x)))*2)+((abs(float(y)))*2))
    
    #factoring the motor direction

#mapping velocity range into motor commands range: velocity: -1 - 3 m/s  motor_commands: -1 - 1:
    if (speed_vector_size * thruster_factor >= 0) :  #deviding into 2 sections. positive speed and negative speed because the system has diffrent limitations in each direction
        motor_value = speed_vector_size / 3
        if (motor_value >=1): #safety procedure in case the given value is our of range
            motor_value = 1
    elif (speed_vector_size * thruster_factor < 0):
        motor_value = (speed_vector_size / -1) * 0.25
        if (motor_value <= -1): #safety procedure in case the given value is our of range
            motor_value = -1

 #mapping angular value from -1.8 rad per second to 1.8 rad per second to -1 to 1 motor values:
    angular_value = abs(float(z) / 1.8)
    if (float(z) >= 0):
        angular_direction_factor = 1
    elif (float(z) < 0):
        angular_direction_factor = -1
    angular_factor = 1 #the ratio between the tourqe the bow thruster can generate and the tourque the back thrusters can generate - on order to rotate around a fixed point
    if (angular_value >=1): #safety procedure in case the given value is our of range
        angular_value  = 1 

    bow_thruster_value = motor_value * math.sin(servo_angle)
    if bow_thruster_value >=1: #safety procedure in case the given value is our of range
        bow_thruster_value=1
    elif bow_thruster_value <=-1: #safety procedure in case the given value is our of range
        bow_thruster_value = -1
    bow_thruster_value=-bow_thruster_value

    print(x)
    print(y)

    #first state - right turn:
    if (x>=0 and z<0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , 70.0, straight , straight, front_angle])

    #second state - left turn:
    elif (x>=0 and z>0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , 140.0, straight , straight, front_angle])

    # third state - revers to the right             
    elif (x<0 and y>0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , 140.0, straight , straight, front_angle])

    # 4th state - revers to the left
    elif (x<0 and y<0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , 70.0, straight , straight, front_angle])

    # 5th state - stop
    elif (x==0 and z==0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , stop_front_motor  , straight , straight, front_angle])

    # turn the boat
    if (z!=0):
        print("turn command")
        print(new_motor_commands)
        try:
            servo_proc.stdin.write("{} {} {} {} {} {}\n".format(str(new_motor_commands[0]),
        str(new_motor_commands[1]),
        str(new_motor_commands[2]),
        str(new_motor_commands[3]),
        str(new_motor_commands[4]),
        str(new_motor_commands[5])).encode())
        except:
            exit(2)    
        #time.sleep(((servo_angle_command/straight)*T)-K)
        #print("entering sleep")
        #time.sleep(T)
        #print("exiting sleep")
    # move forward
    if (x>0):
        new_motor_commands = numpy.array([45.0 , 45.0 , 100.0 , straight , straight , front_angle],dtype=numpy.float32)
    elif (x<0):
        new_motor_commands = numpy.array([135.0 , 135.0 , stop_motor , straight , straight , front_angle],dtype=numpy.float32)
    #elif (x==0):
        #new_motor_commands = numpy.array([stop_motor , stop_motor , stop_front_motor , straight , straight , front_angle],dtype=numpy.float32)
    #converting values from -1 to 1 to 0 -180 which the library work with
    #new_motor_commands = numpy.array([((motor_commands[0]*-90.0)+90.0),((motor_commands[1]*-90.0)+90.0),((motor_commands[2]*-90.0)+90.0),motor_commands[3],motor_commands[4],motor_commands[5] ],dtype=numpy.float32) #need to add
    print("move straight command")
    print(new_motor_commands)
    try:
        servo_proc.stdin.write("{} {} {} {} {} {}\n".format(str(new_motor_commands[0]),
    str(new_motor_commands[1]),
    str(new_motor_commands[2]),
    str(new_motor_commands[3]),
    str(new_motor_commands[4]),
    str(new_motor_commands[5])).encode())
    except:
        exit(2)


def listener():
    rospy.init_node('listener_move_boat', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()



if __name__ == '__main__':
    listener()