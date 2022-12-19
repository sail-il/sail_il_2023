#!/usr/bin/env python3
PKG = 'control'
import gy271compass as GY271
import math, time
 
 
class angular_velocity(object):
 
    def __init__(self) -> None:
        self.delta_time = 0.01
        self.angular_vel = 0
        self.angular_twist = 0
        self.twist_dif_angular_vel = 0
        self.wanted_angular_acc = 0
        self.radian_angle = [0,0]
        self.mass = 35
        self.Ofek_list = [9,188,1,1,3,135]
        self.twist_difz = 0
        self.wanted_acc =0
        self.wanted_thrust_value = 0
        self.practical_thrust = 0
        while True:
            self.testing()
 
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
       
    def angular_velocity(self, delta_time = 0.01):
        self.delta_time = delta_time
        self.angular_vel = self.magneto_angular_velocity() #data from mgnometer sensor
        return self.angular_vel
 
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
   
    def angular_twist_dif2thrust_dif(self):
        self.angular_vel2twist()
        self.twist_dif_angular_vel = self.Ofek_list[5] - self.angular_vel
        self.wanted_angular_acc = (self.twist_dif_angular_vel / self.delta_time)
        if self.wanted_angular_acc < -0.213594313:
            self.wanted_angular_acc = -0.213594313
        else:
            self.wanted_angular_acc = 0.27498907
 
    def wanted_angular_thrust(self):
        self.angular_twist_dif2thrust_dif()
        self.wanted_thrust_value = self.mass * self.wanted_angular_acc
        self.practical_thrust = self.mass * self.angular_velocity() # please check that line :)
 
    def testing(self):
        self.wanted_angular_thrust()
        print(self.practical_thrust, self.wanted_acc, self.angular_vel)
 
if __name__ == '__main__':
  av = angular_velocity()
 

