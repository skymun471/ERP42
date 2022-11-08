#!/usr/bin/env python
# -*- coding: utf-8 -*-

from typing_extensions import Self
import rospy
import numpy as np
import array
import copy
from test_PID import *
#from mi_msgs.msg import *
from erp_driver.msg import erpStatusMsg, erpCmdMsg

class Controller(object):
	def __init__(self):
		self.target_value = erpCmdMsg() # goal speed, goal steering
		self.final_cmd = erpCmdMsg()    # final cmd
		self.car_state = erpCmdMsg()
		self.car_state.speed = 0
		self.remote_control_cmd = erpCmdMsg()
		self.sub_val = 999
		self.target_value.speed = 0
		self.target_value.steer = 0
		self.target_value.gear = 0
		self.current_velocity_value = 0
		self.prev_value = erpCmdMsg()
		self.prev_value.speed = 0
		self.prev_value.steer = 0
		self.prev_value.brake = 0
		self.A_S_controller = Accel_Steer_Controller()
		self.A_S_controller.__init__()
		self.accel_max_change = 2200 # rospy.get_param("~/controller/accel_max_change")
		self.steering_max_change = 2000# rospy.get_param("~/controller/steering_max_change")
		self.brake_max_change = 33 # rospy.get_param("~/controller/brake_max_change")
		self.accel_limiter_max = 200 # rospy.get_param("~/controller/accel_limiter_max")
		self.accel_limiter_min = -2000 # rospy.get_param("~/controller/accel_limiter_min")
		self.brake_limiter_max = 33 # rospy.get_param("~/controller/brake_limiter_max")
		self.brake_limiter_min = 1 # rospy.get_param("~/controller/brake_limiter_min")
		self.brake_gain = 0.5 #0.07	# rospy.get_param("~/controller/brake_gain")
		self.steering_limiter_max = 2000 # rospy.get_param("~/controller/steering_limiter_max")
		self.steering_limiter_min = -2000 # rospy.get_param("~/controller/steering_limiter_min")
		self.brake_cmd = 1

		self.error = 0
		self.control_pub = rospy.Publisher('control_messages', erpCmdMsg, queue_size = 10)
		self.remote_sub = rospy.Subscriber('remote_messages',erpCmdMsg,self.remote_callback)
		self.car_sub = rospy.Subscriber('car_messages',erpCmdMsg,self.car_callback)


	def remote_callback(self, data):
		self.remote_control_cmd = data


	def car_callback(self, data):
		self.car_state = data
		self.current_velocity_value = self.car_state.speed

		if self.remote_control_cmd.gear == 202:
			print("Not ready for auto mode")
			exit()
		else:
			self.target_value = self.remote_control_cmd


		if self.target_value.speed > 50: # velocity limit
			self.target_value.speed = 50

		sub_val = self.target_value.speed - self.current_velocity_value
		#print(self.target_value.velocity, sub_val)
		if self.target_value.speed == 0.0:
			#print('-'*20)
			self.calc()
		else:
			if sub_val < 0:
				#print('='*20)
				self.calc()
				self.sub_val = sub_val
			else:
				if self.sub_val <= 0 and sub_val == self.target_value.speed:
					#print('/'*20)
					pass
				else:
					if self.sub_val < self.target_value.speed and sub_val == self.target_value.speed:
						#print('+'*20)
						pass
					else:
						#print(sub_val)
						self.calc()
						self.sub_val = sub_val

		self.control_publish()


	def brake(self):
		if(self.target_value.speed == 0):
			#self.brake_max_change = rospy.get_param("~brake_max_change") + 200
			self.brake_cmd = 33
			self.accel_cmd = 0
		elif(self.target_value.speed > 0):
			#self.brake_max_change = rospy.get_param("~brake_max_change")
			self.brake_cmd = self.brake_cmd
			self.accel_cmd = self.accel_cmd

	def brake_accel_to_gear_shift(self):
		if(self.error < -10):
			# self.final_cmd.gear = 2
			# self.accel_cmd = abs(self.accel_cmd)

			self.brake_cmd = abs(self.error+10) * self.brake_gain
		# 	self.accel_cmd =0
		# elif(self.error < 0 and self.error >= -10):
		# 	self.accel_cmd =0
		if(self.accel_cmd < 0):
			self.accel_cmd = 0
		if(self.brake_cmd > 1):
			self.accel_cmd = 0
		#return self.accel_cmd, self.brake_cmd

	def velocity(self,t_v, c_v, prev_speed, prev_brake):
		self.target_value.speed = t_v # goal speed
		self.current_velocity_value = c_v #current velocity
		self.prev_value.speed = prev_speed
		self.prev_value.brake = prev_brake
		self.error = t_v - c_v

		# self.accel_limiter_max = min(abs(c_v - t_v)* 2 + 71, 200)
		# self.accel_limiter_max = min(abs(c_v - t_v)* 2 + 55, 200)

		self.accel_cmd = self.A_S_controller.accel_control(self.target_value.speed,self.current_velocity_value)
		rospy.loginfo(self.accel_cmd)
		self.accel_cmd = np.clip(self.accel_cmd,self.accel_limiter_min,self.accel_limiter_max) ###Accel limiter
		if (self.accel_cmd > 0 and self.final_cmd.gear == 2):
			self.final_cmd.gear = 0
		self.brake_accel_to_gear_shift()
		self.brake()
		# rospy.loginfo(self.accel_cmd)
		self.accel_cmd = self.A_S_controller.rate_limiter(self.accel_cmd,self.prev_value.speed,self.accel_max_change)
		self.final_cmd.speed = self.accel_cmd
  		#self.final_cmd.speed = self.accel_cmd
		# self.final_cmd.gear = gear-----
		# rospy.loginfo(self.prev_value.speed)
		# self.prev_value.speed = copy.deepcopy(self.final_cmd.speed)

		#################################brake######################################
		#here
		self.brake_cmd = self.A_S_controller.rate_limiter(self.brake_cmd,self.prev_value.brake,self.brake_max_change)
		self.brake_cmd = np.clip(self.brake_cmd,self.brake_limiter_min, self.brake_limiter_max) ###Brake limiter
		self.final_cmd.brake = self.brake_cmd
		# self.prev_value.brake = copy.deepcopy(self.final_cmd.brake)
		# return thinking
		return [self.accel_cmd, self.final_cmd.gear,  self.final_cmd.brake]

	def steering(self, t_s, c_s, prev_steer):
		self.target_value.steer = t_s # target steering
		self.prev_value.steer = c_s # current steering
		self.steering_cmd = self.A_S_controller.rate_limiter(self.target_value.steer,self.prev_value.steer,self.steering_max_change)
		self.steering_cmd = np.clip(self.steering_cmd, self.steering_limiter_min, self.steering_limiter_max) ###Steering limiter
		self.final_cmd.steer = self.steering_cmd
		self.prev_value.steer = prev_steer

		return self.steering_cmd

	def gear_shift(self):
		self.final_cmd.gear = np.uint8(self.target_value.gear)

	def control_publish(self):
		self.control_pub.publish(self.final_cmd)

	def calc(self):
		self.velocity() ###velocity Command
		self.steering() ###steering Command
		self.gear_shift() ###gear_shift Command

if __name__ == '__main__':
	rospy.init_node('controller')
	controller = Controller()
	rospy.spin()
