#!/usr/bin/env python3
 
PKG = 'control'
from decimal import DivisionByZero
from importlib import import_module
import roslib; roslib.load_manifest(PKG)
import rospy, math, time, sys
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import time
import subprocess
 
servo_proc = subprocess.Popen(["python3.7", "/home/pinksail/control_ws/src/control/src/controller.py"], stdin=subprocess.PIPE)
 
class state_machine(object):
    def _init_(self):
      self.ax = 0
      self.ay = 0
      self.imu_z = 0  
      self.x_vel = 0
      self.y_vel = 0
      self.ang_z_vel = 0
      self.vector_angle_degress = 0
      self.true_angle = 0
      self.servo_angle = 0
      self.servo_angle_command = 0
      self.thruster_factor = 0
      self.speed_vector_size = 0
      self.absolute_speed_vector_size =0
      self.PD_commands = [0,0,0,0,0,0]
      self.state = 0
      self.delta_time = 0
      self.last_time = 0
      self.current_time = time.time()
      self.v0x = 0
      self.v0y = 0
      self.imu_vel_x = 0
      self.imu_vel_y = 0
 
 
      while not rospy.is_shutdown():
        rospy.Subscriber("/imu/data/", Imu, self.imu_callback)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.printer()  
   
    def imu_callback(self, data):
      self.ax = float(data.linear_acceleration.x)
      self.ay = float(data.linear_acceleration.y)
      self.imu_z = float(data.angular_velocity.z)
   
    def cmd_callback(self, cmd_data):
        if (float(cmd_data.linear.x) == 0 and float(cmd_data.linear.y) == 0 and float(cmd_data.angular.z == 0)):
            self.ang_z_vel = float(cmd_data.linear.x)
            self.imu_to_vel()
        else:
            self.x_vel = float(cmd_data.linear.x)
            self.y_vel = float(cmd_data.linear.y)
            self.ang_z_vel = float(cmd_data.angular.z)
 
    def imu_to_vel(self):
        self.delta_time = self.current_time - self.last_time
        self.imu_vel_x = self.v0x + self.ax * self.delta_time
        self.imu_vel_x = self.v0y + self.ax * self.delta_time
        self.last_time = self.current_time
        self.x_vel = -self.imu_vel_x
        self.y_vel = -self.imu_vel_y
 
    def finding_servo_angle(self):
      #finding direction angle from arctan. arctan gives values from -90 to 90 degrees. in order to find the true angle between 0 to 360 the below calculations are needed
      try:
          self.vector_angle_degress = ((math.atan(float(self.y_vel)/float(self.x_vel)))/math.pi)*180
          if (float(self.x_vel) > 0):
              self.true_angle = self.vector_angle_degress
          elif (float(self.x_vel) < 0 and float(self.y_vel) >= 0):
              self.true_angle = self.vector_angle_degress + 180
          elif (float(self.x_vel )< 0 and float(self.y_vel) < 0):
              self.true_angle = self.vector_angle_degress -180
          elif (float(self.x_vel) < 0 and float(self.y_vel >= 0)):
              self.true_angle = self.vector_angle_degress + 180
      except ZeroDivisionError:
          self.true_angle = 0
    #factoring the needed servo angle into angle command between the range of 0 - 180
      if self.true_angle > 90:
          self.servo_angle = self.true_angle - 180
          self.thruster_factor = -1 # the boat cant operate at max reverse speed
      elif self.true_angle < -90:
          self.servo_angle = self.true_angle + 180
          self.thruster_factor = -1 # the boat cant operate at max reverse speed
      else:
          self.servo_angle = self.true_angle
          self.thruster_factor = 1
      #the servo command
      self.servo_angle_command = self.servo_angle + 90
      ###self.speed_vector_size = math.sqrt(((abs(float(self.x_vel)))*2)+((abs(float(y)))*2))
      self.absolute_speed_vector_size = math.sqrt(((abs(float(self.x_vel)))*2)+((abs(float(self.y_vel)))*2))
      self.speed_vector_size = self.absolute_speed_vector_size * self.thruster_factor
   
    def state_chooser(self):
      #first state:
      if ( self.x_vel != 0 and self.y_vel !=0 and self.ang_z_vel == 0):
          self.state = 1
      #second third state - angular moving            
      elif (self.x_vel == 0 and self.y_vel ==0 and  self.ang_z_vel != 0):
        self.state = 2
        if (self.ang_z_vel >= 0):
          self.servo_angle_command = 0
        if (self.ang_z_vel < 0):
          self.servo_angle_command = 180
      #third state - 3 axis movement:
      elif (( self.x_vel != 0 and self.y_vel != 0 and self.ang_z_vel != 0 ) or (self.x_vel == 0 and self.y_vel!=0 and self.ang_z_vel!=0) or (self.x_vel != 0 and self.y_vel == 0 and self.ang_z_vel != 0)):
          self.state = 3
      self.PD_commands = [self.speed_vector_size , self.state ,self.servo_angle_command, self.ax , self.ay , self.ang_z_vel]
 
    def send_command(self):
      try:
          servo_proc.stdin.write("{} {} {} {} {} {}\n".format(str(self.PD_commands[0]),
          str(self.PD_commands[1]),
          str(self.PD_commands[2]),
          str(self.PD_commands[3]),
          str(self.PD_commands[4]),
          str(self.PD_commands[5])).encode())
      except:
          exit(2)
 
    def printer(self):
      self.state_chooser()
      self.finding_servo_angle()
      print("the imu list is: ", self.imu_list, '\n', "the cmd list is: ", self.cmd_list, '\n',"the speed vector size is: ", self.speed_vector_size , '\n', "the servo angle command is: ", self.servo_angle_command, '\n', "the PD list is: ",self.PD_commands, '\n')
 
 
if __name__ == '_main_':
  rospy.init_node('listener', anonymous=True)
  b = state_machine()


