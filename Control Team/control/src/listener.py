#!/usr/bin/env python
PKG = 'control'
import roslib; roslib.load_manifest(PKG)
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import subprocess

servo_proc = subprocess.Popen(["python3.7", "/home/pinksail/control_ws/src/control/src/python3code.py"], stdin=subprocess.PIPE)

#converts the list message to python 3.7 format
#because ros runs on python2 but circuitpython runs on python 3
def callback(data):
    print("Received command: {}".format(data))
    motor_commands = data.data
    try:
        servo_proc.stdin.write("{} {} {} {} {} {}\n".format(str(motor_commands[0]),str(motor_commands[1]),str(motor_commands[2]),str(motor_commands[3]),str(motor_commands[4]),str(motor_commands[5])).encode())
    except:
        exit(2)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()