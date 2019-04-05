#!/usr/bin/env python


import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
import numpy as np
import tf
import time


class Autonumos():
	def __init__(self):
		self.index= 0 
		self.is_on = False
		self.action_order = ["auto","auto","image"]
class robot_state():
	def __init__(self):
		self.image_processing = False
	    self.autonumos = Autonumos()
	    self.autonumos.is_on = False
	    self.teleop = False
	def status_callback(self,status):
		if status=='teleop':
			self.teleop = True
			self.autonumos.is_on  = False
			self.image_processing = False
		elif status == 'fms':
			self.teleop = False
			self.autonumos.is_on  = True
			self.image_processing = False
		elif status == 'auto':
			self.teleop = False
			self.autonumos.is_on  = False
			self.image_processing = True
	def path_follower_status_callback(self,status):
		if self.autonumos.is_on and status == "done":
			self.autonumos.index+=1
			

if __name__ == '__main__':
	rospy.init_node('state_machine')
	rate = rospy.Rate(50)
	state = robot_state()
	cmd_vel_sub = rospy.Subscriber('/robot_status', String, state.status_callback)
	while not rospy.is_shutdown():
		pass

