#!/usr/bin/env python
PKG = 'control'
from cmath import sqrt
#from this import s
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

decision_distance = 4.0
too_close_distance = 0.8
long_decision_angle = 45.0
no_turn_long_angle = 5.0
short_decision_angle = 30.0
no_turn_short_angle = 10.0
straight=90.0
stop_motor=94.0
stop_front_motor=94.0
front_motor_correction = 105.0
right_turn=70.0
left_turn=155.0
T=30.0
K=0.2
strong_motor = 45.0
medium_motor = 60.0
week_motor = 75.0
crab_right_angle = 180.0
crab_left_angle = 0.0 
cannon_on = 5
cannon_off = 6

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
        if (x>=0):
            true_angle = 90
        else:
            true_angle = 270
    ## calc distance 
    goal_dist = math.sqrt(x**2+y**2)

    no_turn = 0
    turn_while_sail = 0
## choose by distance ##
    if (goal_dist >= decision_distance):
        ## decide by angle
        if (abs(true_angle-straight) <= long_decision_angle and abs(true_angle-straight) >= no_turn_long_angle):
            turn_while_sail = 1
        if (abs(true_angle-straight)<no_turn_long_angle):
            no_turn = 1
    elif (goal_dist < decision_distance and goal_dist > too_close_distance):
        if (abs(true_angle-straight) <= short_decision_angle and abs(true_angle-straight) >= no_turn_short_angle):
            turn_while_sail = 1
        if (abs(true_angle-straight)<no_turn_short_angle):
            no_turn = 1
    elif (goal_dist < too_close_distance):
        no_turn = 1

## crab movment
    thrust_angle = straight
    if (z == 1):
        if (y>0):
            thrust_angle = crab_right_angle
        elif(y<0):
            thrust_angle = crab_left_angle  
    
    front_motor_turn = stop_motor
## front motor command
    if(y>0):
        front_motor_turn = right_turn
    elif(y<0):
        front_motor_turn = left_turn

## cannon control
    cannon_control = 0
    if (z == cannon_on): 
        cannon_control = cannon_on
    if (z == cannon_off):
        cannon_control = cannon_off

##crab move
    #if (z == 1):
        

## move only forward
    if (no_turn):
        if (goal_dist >= decision_distance):
            new_motor_commands = numpy.array([strong_motor , strong_motor , front_motor_correction , thrust_angle , thrust_angle , cannon_control],dtype=numpy.float32)
        elif (goal_dist < decision_distance and goal_dist > too_close_distance):
            new_motor_commands = numpy.array([medium_motor , medium_motor , front_motor_correction , thrust_angle , thrust_angle , cannon_control],dtype=numpy.float32)
        elif (goal_dist < too_close_distance):
            new_motor_commands = numpy.array([week_motor , week_motor , stop_motor , thrust_angle , thrust_angle , cannon_control],dtype=numpy.float32)
    
## move forward and turn   
    elif(turn_while_sail):
        if (goal_dist >= decision_distance):
            new_motor_commands = numpy.array([strong_motor , strong_motor , front_motor_turn , thrust_angle , thrust_angle , cannon_control],dtype=numpy.float32)
        elif (goal_dist < decision_distance and goal_dist > too_close_distance):
            new_motor_commands = numpy.array([medium_motor , medium_motor , front_motor_turn , thrust_angle , thrust_angle , cannon_control],dtype=numpy.float32)
        elif (goal_dist < too_close_distance):
            new_motor_commands = numpy.array([week_motor , week_motor , front_motor_turn , thrust_angle , thrust_angle , cannon_control],dtype=numpy.float32)
    
## turn on the spot
    elif(y!=0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , front_motor_turn, thrust_angle , thrust_angle, cannon_control])
    elif(x==0 and y==0):
        new_motor_commands = numpy.array([stop_motor , stop_motor , stop_front_motor  , thrust_angle , thrust_angle, cannon_control])

    if(z==8):
        new_motor_commands = numpy.array([75.0,115.0,94.0,90.0,90.0,5])
    print("move boat")
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


## listener node ##
def listener():
    rospy.init_node('listener_move_boat', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()


## main ##
if __name__ == '__main__':
    listener()
