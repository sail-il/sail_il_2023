#!/usr/bin/env python3
PKG = 'control'
from asyncio import FIRST_COMPLETED
from ctypes import cdll
from decimal import DivisionByZero
from importlib import import_module
from os import lstat
from re import T, X
from this import s
import roslib; roslib.load_manifest(PKG)
import rospy, math, time, sys
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import warnings
import gy271compass as GY271
import subprocess
 
servo_proc = subprocess.Popen(["python3.7", "/home/pinksail/control_ws/src/control/src/controller.py"], stdin=subprocess.PIPE)
 
class worker(object):
  def __init__(self):
   
    self.frequency = 50 #50 Hz
    self.delta_time = 1.0 / self.frequency
    self.x = 0
    self.y = 0
    self.z = 0
    self.lin_vel = 0
    self.xyz_list = [0,0,0]
    self.Ofek_list = [0,0,0,0,0]
    self.mass = 35 # assuming boat weights 35kg
    self.wanted_acc = 0
    self.wanted_angular_thrust_value = 0
    self.pid_thrust_value = 0
    self.servo_value = 0
    self.new_commands = [0,0,0,0,0]
    self.lin_proportional = 0
    self.lin_derivitive = 0
    self.lin_integral = 0
    self.Kp1, self.Ki1, self.Kd1 = 5,0,15    #parameters for PID of linear vectors
    self.Kp2, self.Ki2, self.Kd2 = 215,0,0.2 #parameters for PID of linear vectors
    self.Kp3, self.Ki3, self.Kd3 = 11,0,11   #parameters for PID of linear vectors
    self.ang_Kp1, self.ang_Ki1, self.ang_Kd1 = 5,15 ,0   #parameters for PID of bow_thruster vectors
    self.ang_Kp2, self.ang_Ki2, self.ang_Kd2 = 215,0.2,0 #parameters for PID of bow_thruster vectors
    self.ang_Kp3, self.ang_Ki3, self.ang_Kd3 = 11,11,0   #parameters for PID of bow_thruster vectors
    self.angular_error = 0
    self.lin_error = 0
    self.angle = 0
    self.radian_angle = [0,0]
    self.angular_vel = 0
    self.angular_twist = 0
    self.twist_dif_angular_vel = 0
    self.wanted_angular_acc = 0
    self.angular_practical_thrust = 0
    self.pid_bowthrust_value = 0
    self.lin_ang_proportional = 0
    self.lin_ang_derivitive = 0
    self.lin_ang_integral = 0
 
    while not rospy.is_shutdown():
      rospy.Subscriber("/imu/data/", Imu, self.imu_callback)
      rospy.Subscriber("state_machine", numpy_msg(Floats), self.state_callback)
      self.tester()
 
  def imu_callback(self, imu_data):
    self.x = float(imu_data.linear_acceleration.x)
    self.y = float(imu_data.linear_acceleration.y)
    self.z = float(imu_data.angular_velocity.z)
    self.xyz_list = [self.x , self.y, self.z]
   
  def state_callback(self, Ofek_data):
  # for line in sys.stdin:
  #   line = line.split(" ")
  #   state_callback([
  #       float(line[0]),
  #       float(line[1]),
  #       float(line[2]),
  #       float(line[3]),
  #       float(line[4]),
  #       float(line[5])])
    PD_commands = Ofek_data.data
    self.wanted_lin_vel = PD_commands[0]
    self.wanted_ang_vel = PD_commands[1]
    self.state = PD_commands[2]
    self.needed_servo_degree = PD_commands[3]
    self.Ofek_list =  [self.wanted_lin_vel, self.wanted_ang_vel, self.state, self.needed_servo_degree]
 
  ###################      velocities         ######################
 
  def linear_velocity(self): # linear velocity function
    prac_acc = np.sqrt( (self.xyz_list[0])**2 + (self.xyz_list[1])**2)
    if self.xyz_list[0] > 0:    #if acceleration is positive
        self.lin_vel = self.lin_vel + prac_acc * self.delta_time
   
    else:                 #if acceleration is negative
        self.lin_vel = self.lin_vel - prac_acc * self.delta_time
    return self.lin_vel
 
  def magneto_angular_velocity(self): # Angular velocity function with magnetometer data
    sensor = GY271.compass(address=0x0d)
    time.sleep(1)
    self.angle = sensor.get_bearing()
    self.radian_angle[1] = (self.angle * math.pi) / 180 # radians conversion
    angular_velocity = (self.radian_angle[1] - self.radian_angle[0])/self.delta_time
    self.radian_angle[0] = self.radian_angle[1]
    #print(f"angular velocity is: {angular_velocity} radians per second")
    #print('Heading Angle = {}°'.format(angle))
    return float(angular_velocity)
 
  def angular_velocity(self):
      self.angular_vel = self.magneto_angular_velocity() #data from mgnometer sensor
      return self.angular_vel
 
  ###################      twists       ######################
 
  def lin_vel2twist(self): # switch from velocity to Twist, assuming boat weights 35kg, Cd = 0.1 due to shape of hulls, A = 1.72m^2, density of water = 1
    self.linear_velocity()
    if self.lin_vel > 0:
        self.lin_twist = (-0.0002) * (self.lin_vel**3) + 0.0076 * (self.lin_vel**2) + 0.0339 * (self.lin_vel) + 0.0411
        if self.lin_twist > 1:
          self.lin_twist = 1
    elif self.lin_vel < 0:
        self.lin_twist = (-0.0003) * (self.lin_vel**3) - 0.0091 * (self.lin_vel**2) + 0.04 * self.lin_vel - 0.0478
        if self.lin_twist < -1:
          self.lin_twist = -1
    else:
        self.lin_twist = 0
 
  def angular_vel2twist(self):
      self.angular_velocity()
      if self.angular_vel > 0:
          self.angular_twist = (-0.0002) * (self.angular_vel**3) + 0.0076 * (self.angular_vel**2) + 0.0339 * (self.angular_vel) + 0.0411
      if self.angular_twist > 1:
          self.angular_twist = 1
      elif self.angular_vel < 0:
          self.angular_twist = (-0.0003) * (self.angular_vel**3) - 0.0091 * (self.angular_vel**2) + 0.04 * self.angular_vel - 0.0478
          if self.angular_twist < -1:
              self.angular_twist = -1
      else:
          self.angular_twist = 0
 
  ###################      thrusts        ######################
 
  def twist_dif2thrust_dif(self):
    self.lin_vel2twist()
    self.twist_difx = self.Ofek_list[0] - self.lin_vel
    self.wanted_acc = (self.twist_difx / self.delta_time)
    if self.wanted_acc < -0.213594313:
      self.wanted_acc = -0.213594313
    elif self.wanted_acc > 0.27498907:
      self.wanted_acc = 0.27498907
    else:
      pass
 
  def wanted_lin_thrust(self):
    self.twist_dif2thrust_dif()
    self.wanted_lin_thrust_value = self.mass * self.wanted_acc
    self.practical_thrust = self.mass * self.linear_velocity()
 
  def angular_twist_dif2thrust_dif(self):
      self.angular_vel2twist()
      self.twist_dif_angular_vel = self.Ofek_list[1] - self.angular_vel
      self.wanted_angular_acc = (self.twist_dif_angular_vel / self.delta_time)
      if self.wanted_angular_acc < -0.213594313:
        self.wanted_angular_acc = -0.213594313
      elif self.wanted_angular_acc > 0.27498907:
        self.wanted_angular_acc = 0.27498907
      else:
        pass
 
  def wanted_angular_thrust(self):
    self.angular_twist_dif2thrust_dif()
    self.wanted_angular_thrust_value = self.mass * self.wanted_angular_acc
    self.angular_practical_thrust = self.mass * self.angular_velocity()
 
     
 
  ###################      PID         ######################
 
  def lin_pid_state_filter(self):
    self.wanted_lin_thrust()
    self.lin_error = self.wanted_lin_thrust_value - self.practical_thrust
   
    if self.Ofek_list[2] == 1: #straight line
      self.lin_proportional += self.Kp1 * self.lin_error
      self.lin_derivitive   += self.Kd1 * self.lin_error / self.delta_time
      self.lin_integral     += self.Ki1 * self.lin_error * self.delta_time
      self.pid_thrust_value = self.lin_integral + self.lin_derivitive + self.lin_proportional
   
    elif self.Ofek_list[2] == 2: #angular movement
      self.lin_proportional += self.Kp2 * self.lin_error
      self.lin_derivitive   += self.Kd2 * self.lin_error / self.delta_time
      self.lin_integral     += self.Ki2 * self.lin_error * self.delta_time
      self.pid_thrust_value = self.lin_integral + self.lin_derivitive + self.lin_proportional
   
    elif self.Ofek_list[2] == 3: # combined  movement
      self.lin_proportional += self.Kp3 * self.lin_error
      self.lin_derivitive   += self.Kd3 * self.lin_error / self.delta_time
      self.lin_integral     += self.Ki3 * self.lin_error * self.delta_time
      self.pid_thrust_value = self.lin_integral + self.lin_derivitive + self.lin_proportional      
 
  def angular_pid_state_filter(self):
    self.wanted_angular_thrust()
    self.angular_error = self.wanted_angular_thrust_value - self.practical_thrust
   
    if self.Ofek_list[2] == 1: #straight line
      self.lin_ang_proportional += self.ang_Kp1 * self.angular_error
      self.lin_ang_derivitive   += self.ang_Kd1 * self.angular_error / self.delta_time
      self.lin_ang_integral     += self.ang_Ki1 * self.angular_error * self.delta_time
      self.pid_bowthrust_value = self.lin_integral + self.lin_derivitive + self.lin_proportional
   
    elif self.Ofek_list[2] == 2: #angular movement
      self.lin_ang_proportional += self.ang_Kp2 * self.angular_error
      self.lin_ang_derivitive   += self.ang_Kd2 * self.angular_error / self.delta_time
      self.lin_ang_integral     += self.ang_Ki2 * self.angular_error * self.delta_time
      self.pid_bowthrust_value = self.lin_integral + self.lin_derivitive + self.lin_proportional
   
    elif self.Ofek_list[2] == 3: # combined  movement
      self.lin_ang_proportional += self.ang_Kp3 * self.angular_error
      self.lin_ang_derivitive   += self.ang_Kd3 * self.angular_error / self.delta_time
      self.lin_ang_integral     += self.ang_Ki3 * self.angular_error * self.delta_time
      self.pid_bowthrust_value = self.lin_integral + self.lin_derivitive + self.lin_proportional
     
 
  ###################      servos         ######################
 
  def lin_thrust2servo(self):
    self.lin_pid_state_filter()
    if self.pid_thrust_value < -0.213594313:   #clamp thrust value to not surpass engines ability
      self.pid_thrust_value = -0.213594313
    elif self.pid_thrust_value > 0.27498907:
      self.pid_thrust_value = 0.27498907
    else:
      pass
    if float(self.pid_thrust_value) < 0:       #converts thrust value to servo value that can be sent to PCA
        self.servo_value = 0.1154 * (self.pid_thrust_value**3) + 1.8574 * (self.pid_thrust_value**2) + 18.299 * (self.pid_thrust_value) + 82.021
    elif float(self.pid_thrust_value) > 0:
        self.servo_value = 0.0532 * (self.pid_thrust_value**3) - 1.1336  * (self.pid_thrust_value**2) + 14.395 * (self.pid_thrust_value) + 97.265
    else:
        self.servo_value = 90
 
  def bow_thrust2servo(self):
    self.angular_pid_state_filter()
    if self.pid_bowthrust_value < -0.213594313:  #clamp thrust value to not surpass engines ability
      self.pid_bowthrust_value = -0.213594313
    elif self.pid_bowthrust_value > 0.27498907:
      self.pid_bowthrust_value = 0.27498907
    else:
      pass
    if float(self.pid_bowthrust_value) < 0:    #converts thrust value to servo value that can be sent to PCA
        self.bow_servo_value = 0.1154 * (self.pid_bowthrust_value**3) + 1.8574 * (self.pid_bowthrust_value**2) + 18.299 * (self.pid_bowthrust_value) + 82.021
    elif float(self.pid_bowthrust_value) > 0:
        self.bow_servo_value = 0.0532 * (self.pid_bowthrust_value**3) - 1.1336  * (self.pid_bowthrust_value**2) + 14.395 * (self.pid_bowthrust_value) + 97.265
    else:
        self.bow_servo_value = 90
 
 
  ###################      new command         ######################
 
  def thrusters_servo_value(self):
    self.bow_thrust2servo
    self.lin_thrust2servo()
    self.new_commands = [self.servo_value, self.servo_value, self.bow_servo_value, self.Ofek_list[3], self.Ofek_list[3]]
    return self.new_commands
 
  ###################      tester       ######################
 
  def tester(self):
    self.thrusters_servo_value()
    #print("the twist value is: ", self.lin_twist, '\n', "the acceleration value is: ", self.wanted_acc, '\n',"the thrust value is: ", self.wanted_thrust_value, '\n',"the thruser servo value being sent is: ", self.servo_value,'\n', "the thruster pid value is: ", self.pid_thrust_value,'\n', 'the new comands are: ', self.new_commands, '\n')
 
 
 
if __name__ == '__main__':
  rospy.init_node('listener', anonymous=True)
  a = worker()
 
 

