#!/usr/bin/env python
PKG = 'control'
import roslib; roslib.load_manifest(PKG)

import rospy
import pygame
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
#from motor_initialise import motor_initialise
import numpy

def init():
    pygame.init()
    win = pygame.display.set_mode((250,250))

def getkey(keyname):
    ans = False
    for eve in pygame.event.get():pass
    keyinput = pygame.key.get_pressed()
    mykey = getattr(pygame, 'K_{}'.format(keyname))
    if keyinput [mykey]:
        ans = True
    pygame.display.update()
    return ans
    
def true_talker():
    pub = rospy.Publisher('floats', numpy_msg(Floats), queue_size=10)
    rospy.init_node('talker', anonymous=False)
    r= rospy.Rate(1) #10hz
    #[right_thruster, left_thruster, bow_thruster ,right_servo , left_servo, special_stuff ]

    while not rospy.is_shutdown():
        if getkey('w'):
            motor_commands = numpy.array([0.3,0.3,0.0,90.0,90.0,0],dtype=numpy.float32)
        elif getkey('s'):
            motor_commands = numpy.array([-0.3,-0.3,0.0,90.0,90.0,0],dtype=numpy.float32)
        elif getkey('a'):
            motor_commands = numpy.array([0.25,0.25,0.3,180.0,180.0,0],dtype=numpy.float32)
        elif getkey('d'):
            motor_commands = numpy.array([0.25,0.25,-0.3,0.0,0.0,0],dtype=numpy.float32)
        elif getkey('k'): #off kill switch
            motor_commands = numpy.array([0.11,0.11,0.11,90,90,1],dtype=numpy.float32)
        elif getkey('l'): #on kill switch
            motor_commands = numpy.array([0.11,0.11,0.11,90,90,2],dtype=numpy.float32)
        elif getkey('n'): #switch to 18.5v battery
            motor_commands = numpy.array([0.11,0.11,0.11,90,90,4],dtype=numpy.float32)
        elif getkey('m'): #switch to 14.8v
            motor_commands = numpy.array([0.11,0.11,0.11,90,90,3],dtype=numpy.float32)
        
        else:
            motor_commands = numpy.array([0.11,0.11,0.11,90,90,0],dtype=numpy.float32) # 0.11 should stop the motors for the current configuration
        
        new_motor_commands = numpy.array([((motor_commands[0]*-90.0)+90.0),((motor_commands[1]*-90.0)+90.0),((motor_commands[2]*-90.0)+90.0),motor_commands[3],motor_commands[4], motor_commands[5] ],dtype=numpy.float32)
        pub.publish(new_motor_commands)
    
        r.sleep()

if __name__ == '__main__':
    init()
    true_talker()
