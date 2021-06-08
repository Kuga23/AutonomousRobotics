#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
	def __init__(self):
		self.steering={}
		self.drive={}
		for k in prefix:
			self.steering[k]=0.0
			self.drive[k]=0.0
	def copy(self,value):
		for k in prefix:
			self.steering[k]=value.steering[k]
			self.drive[k]=value.drive[k]

class DriveConfiguration:
	def __init__(self,radius,x,y,z):
		self.x = x
		self.y = y
		self.z = z
		self.radius = radius


class RoverKinematics:
	def __init__(self):
		self.X = numpy.asmatrix(numpy.zeros((3,1)))
		self.motor_state = RoverMotors()
		self.first_run = True

	def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
		motors = RoverMotors()
		dist = lambda a, b: min(abs(a-b), abs(2*pi-(a-b)))
		if skidsteer:
			for k in drive_cfg.keys():
				# Insert here the steering and velocity of 
				# each wheel in skid-steer mode     
				if k[1] == "L":
					vx = twist.linear.x - twist.angular.z*drive_cfg["CL"].y
					vy = twist.linear.y + twist.angular.z*drive_cfg["CL"].x
					beta = atan2(vy, vx)
					motors.steering[k] = 0
					motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
					if(twist.linear.x<0):
						motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
				if k[1] == "R":
					vx = twist.linear.x - twist.angular.z*drive_cfg["CR"].y
					vy = twist.linear.y + twist.angular.z*drive_cfg["CR"].x
					motors.steering[k] = 0
					motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
					if(twist.linear.x<0):
						motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
				
				
						  
		else:
			for k in drive_cfg.keys():
				#print("%s: %f %f"%(k,drive_state.steering[k],drive_state.drive[k]))
				# Insert here the steering and velocity of 
				# each wheel in rolling-without-slipping mode
				vx = twist.linear.x - twist.angular.z*drive_cfg[k].y
				vy = twist.linear.y + twist.angular.z*drive_cfg[k].x
				#if(drive_state.drive[k]-)
				beta = atan2(vy, vx)
				
				if drive_state is not None:
					#i = numpy.argmin([dist(drive_state.steering[k]%(2*pi), beta%(2*pi)), dist(drive_state.steering[k]%(2*pi),(beta+pi)%(2*pi))])
					bpp = (beta+pi)%(2*pi) if (beta+pi)%(2*pi) < pi else 2*pi-(beta+pi)%(2*pi)
					i = 0 if abs(beta-drive_state.steering[k])%(2*pi) < pi/2 else 1
					motors.steering[k] = beta  if (i==0) else bpp # (beta, bpp)[i] # between pi and -pi
					motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
					motors.drive[k] *= 1 - 2*(i==1)
					#print("i == 1: %d, (b, b_): (%f, %f), v: %f"%(i==1, beta, drive_state.steering[k], motors.drive[k]))
				else :
					motors.steering[k] = beta # between pi and -pi
					motors.drive[k] = hypot(vy, vx)/drive_cfg[k].radius
				
			
		return motors

	def integrate_odometry(self, motor_state, drive_cfg):
		# The first time, we need to initialise the state
		if self.first_run:	
			self.motor_state.copy(motor_state)
			self.first_run = False
			return self.X
		# Insert here your odometry code
		dist = lambda a, b: min(abs(a-b), abs(2*pi-(a-b)))
		B = []
		A = []
		for k in prefix:
			#print(k, " : ", motor_state.drive[k])
			delta_drive = dist(motor_state.drive[k]%(2*pi), self.motor_state.drive[k]%(2*pi))*drive_cfg[k].radius
			delta_steer = dist(motor_state.steering[k]%(2*pi), self.motor_state.steering[k]%(2*pi))
			B.append(delta_drive*cos(delta_steer))
			B.append(delta_drive*sin(delta_steer))
			
			A.append([1, 0, -drive_cfg[k].y])
			A.append([0, 1, drive_cfg[k].x])
		B = numpy.array(B)
		A = numpy.array(A)
		B = B.reshape([B.shape[0]] + [1])
		#print("A shape ", A.shape, "B shape ", B.shape)
		delta = numpy.matmul(numpy.linalg.inv(numpy.matmul(A.T, A)), numpy.matmul(A.T, B))
		
		self.X[0,0] += delta[0]*sin(delta[2]) 
		self.X[1,0] += delta[1]*cos(delta[2])
		self.X[2,0] += -delta[2]
		#print("delta shape ", delta, delta.shape, "X ", self.X)
		self.motor_state.copy(motor_state)
		return self.X



