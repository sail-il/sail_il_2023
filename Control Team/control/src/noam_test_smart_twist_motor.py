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
from numpy_true_talker import getkey

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
        vector_angle_degress = ((math.atan(float(y)/float(x)))/math.pi)*180
        if (float(x) > 0):
            true_angle = vector_angle_degress
        elif (float(x) < 0 and float(y) >= 0):
            true_angle = vector_angle_degress + 180
        elif (float(x )< 0 and float(y) < 0):
            true_angle = vector_angle_degress -180
        elif (float(x) < 0 and float(y >= 0)):
            true_angle = vector_angle_degress + 180
    except ZeroDivisionError: 
        true_angle = 0

#factoring the needed servo angle into angle command between the range of 0 - 180
    if true_angle > 90:
        servo_angle = true_angle - 180
        thruster_factor = -0.25 # the boat cant operate at max reverse speed
    elif true_angle < -90:
        servo_angle = true_angle + 180
        thruster_factor = -0.25 # the boat cant operate at max reverse speed
    else:
        servo_angle = true_angle
        thruster_factor = 1

    #speed vector size:
    speed_vector_size = math.sqrt(((abs(float(x)))**2)+((abs(float(y)))**2))
    #the servo command
    servo_angle_command = servo_angle + 90
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
    angular_factor = 0.5 #the ratio between the tourqe the bow thruster can generate and the tourque the back thrusters can generate - on order to rotate around a fixed point
    if (angular_value >=1): #safety procedure in case the given value is our of range
        angular_value  = 1 

    bow_thruster_value = motor_value * math.sin(servo_angle)
    if bow_thruster_value >=1: #safety procedure in case the given value is our of range
        bow_thruster_value=1
    elif bow_thruster_value <=-1: #safety procedure in case the given value is our of range
        bow_thruster_value = -1

    #first state:
    if (x!=0 and y!=0 and z==0): #or (x!=0 and y==0 and z==0) or (x==0 and y!=0 and z==0)):
        motor_commands = numpy.array([motor_value , motor_value , bow_thruster_value/angular_factor , bow_thruster_value/angular_factor , servo_angle_command , servo_angle_command])
    #second state - 3 axis movement:
    elif (x!=0 and y!=0 and z!=0): #or (x==0 and y!=0 and z!=0) or (x!=0 and y==0 and z!=0)):
            if (((bow_thruster_value / angular_factor) + angular_direction_factor * angular_value) >= 1) :
                motor_commands = numpy.array([motor_value , motor_value , 1 ,servo_angle_command , servo_angle_command, 0])
            elif (((bow_thruster_value / angular_factor) + angular_direction_factor * angular_value) <= -1) :
                motor_commands = numpy.array([motor_value , motor_value , -1 ,servo_angle_command , servo_angle_command, 0])
            else:
                motor_commands = numpy.array([motor_value , motor_value , 
                ((bow_thruster_value / angular_factor) + angular_direction_factor * angular_value) , 
                  servo_angle_command , servo_angle_command, 0])
    # third state - angular moving            
    elif (x==0 and y==0 and z!=0):
            if (z >= 0):
                motor_commands =numpy.array([angular_value * angular_factor , angular_value * angular_factor , -angular_value , 0 , 0,0])
            if (z < 0):
                motor_commands =numpy.array([angular_value * angular_factor , angular_value * angular_factor , -angular_value , 180 , 180, 0])
    #print(motor_commands)
    #converting values from -1 to 1 to 0 -180 which the library work with
    new_motor_commands =numpy.array([((motor_commands[0]*-90.0)+90.0),((motor_commands[1]*-90.0)+90.0),((motor_commands[2]*-90.0)+90.0),180-motor_commands[3],180-motor_commands[4],motor_commands[5] ],dtype=numpy.float32) #need to add
    
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
