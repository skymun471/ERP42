#!/usr/bin/env python

import rospy
import numpy as np

class Accel_Steer_Controller(object):
    def __init__(self):
        self.Kp = 1.5# 3.5# 15.0
        self.P = 0
        self.Ki_plus = 15# 1 #0.8
        self.Ki_minus = 0.0003
        self.I = 0
        self.Kd = 400
        self.D = 0
        self.target_velocity = 0
        self.current_velocity = 0
        self.current_error = 0
        self.prev_error = 0
        self.accel_cmd = 0


    def pid_control(self):
        self.P = self.Kp * self.current_error
        if(self.current_error > 0):
            self.I = self.I + self.current_error*self.Ki_plus # update I
        else:
            self.I = self.I + self.current_error*self.Ki_minus
        self.I = np.clip(self.I, 0, 85)
        #self.I = max(self.I, 0)
        self.D = (self.current_error - self.prev_error) * self.Kd
        if(self.current_error < 0):
            self.D = 0
            self.P = 0
        self.accel_cmd = self.P + self.I + self.D # update accel_cmd
        # rospy.loginfo(self.I)


    def accel_control(self,target_velocity_,current_velocity_):
        self.target_velocity = target_velocity_
        self.current_velocity = current_velocity_
        self.current_error = (self.target_velocity - self.current_velocity)
        self.prev_error = self.current_error
        self.pid_control()


        #print("Accel cmd : ", self.accel_cmd)
        return self.accel_cmd


    def rate_limiter(self,value,prev_value,max_change):
        delta = value - prev_value
        delta = np.clip(delta,-max_change,max_change)
        #print(value, prev_value, delta, 1212121212)

        return prev_value + delta
