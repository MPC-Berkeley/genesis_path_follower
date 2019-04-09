#!/usr/bin/env python
import rospy  ##TODO: Cannot find rospy
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import vehicle_state
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from lk_utils.controllers import *
from lk_utils.path_lib import *
from lk_utils.vehicle_lib import *
from lk_utils.velocityprofiles import *
from lk_utils.sim_lib import *
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import pickle
import time


#################################################################################################
### Lanekeeping controller from Nitin Kapania's thesis
#################################################################################################

class LanekeepingPublisher():

	'''
	Publishes acceleration and steering commands based on LMPC
	'''

	def __init__(self):

		#Initialize nodes, publishers, and subscribers
		rospy.init_node('lk_cmd_pub', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.parseStateEstMessage, queue_size=2)

		self.steer_pub = rospy.Publisher("/control/steer_angle", Float32, queue_size = 2)

		self.enable_acc_pub   = rospy.Publisher("/control/enable_accel", UInt8, queue_size =2, latch=True)  ##Why queue_size = 10?
		self.enable_steer_pub = rospy.Publisher("/control/enable_spas",  UInt8, queue_size =2, latch=True)
		self.vehiclestate_pub = rospy.Publisher("/vehicle/vehicle_state", vehicle_state, queue_size = 2, latch = True)


		self.rateHz = rospy.get_param('rate')
		self.rate = rospy.Rate(self.rateHz)  ##TODO: Can we run this fast?

		#Initialize Path object
		self.path = Path()
		if not rospy.has_param('mat_waypoints'):
			raise ValueError('Invalid rosparam global origin provided!')	

		pathLocation = rospy.get_param('mat_waypoints')	
		self.path.loadFromMAT(pathLocation)
		self.trackLength = self.path.s[-1];

		#vehicle information needed - initialize to none
		self.X = self.path.posE[0] 
		self.Y = self.path.posN[0]
		self.psi = self.path.roadPsi[0]
		self.Ux = 0.
		self.Ax = 0.
		self.delta = 0.
		self.accelMax = 2.0
		self.accelMin = 5.0 #negative value implied by LMPC controller

		#Initialize vehicle
		self.genesis = Vehicle('genesis')

		#Create speed profile - choose between constant velocity limit or track-varying velocity limit
		self.speedProfile  = BasicProfile(self.genesis, self.path, friction = 0.4, vMax = 15., AxMax = 2.0)

		#Create controller object - use lanekeeping
		self.controller = LaneKeepingController(self.path, self.genesis, self.speedProfile)

		#Create local and global states and control input object
		self.localState   = LocalState()
		self.globalState  = GlobalState(self.path)
		self.controlInput = ControlInput()
		
		#Initialization Parameters for LMPC controller; 
		
		#Initialize map matching object - use closest style
		self.mapMatch = MapMatch(self.path, "embed")
		

		#Enable steering
		self.enable_steer_pub.publish(0) # enable steering control.
		self.enable_acc_pub.publish(0) # enable acceleration control.

		self.pub_loop()


	def parseStateEstMessage(self, msg):
		self.X = msg.x
		self.Y = msg.y
		self.psi = msg.psi
		self.Ux = msg.v
		self.Ax = msg.a
		self.delta =  msg.df
		#self.Uy = msg.vy #switching from Borrelli's notation to Hedrick's
		#self.Ux = msg.vx #switching from Borrelli's notation to Hedrick's
		#self.r = msg.wz  #switching from Borrelli's notation to Hedrick's

	def pub_loop(self):
		#Start testing!
		t_start = rospy.Time.now()
		print('Path Tracking Test: Started at %f' % (t_start.secs + t_start.nsecs*1e-9))
		while not rospy.is_shutdown():
			t_now = rospy.Time.now()
			dt = t_now - t_start
			dt_secs = dt.secs + 1e-9 * dt.nsecs

			#Populate localState and globalState objects from state publisher
			self.localState.update(Ux = self.Ux)
			self.globalState.update(posE = self.X, posN = self.Y, psi = self.psi)
			self.mapMatch.localize(self.localState, self.globalState)

		
			#Localize Vehicle
			
			#print("Lateral Error is " + str(self.localState.e) )
			
			self.controller.updateInput(self.localState, self.controlInput)
			delta = self.controlInput.delta
			Fx = self.controlInput.Fx

			# use F = m*a to get desired acceleration. Limit acceleration command to 2 m/s
			accel = min( Fx / self.genesis.m , self.accelMax)
			
			self.steer_pub.publish(delta)
			#self.accel_pub.publish(accel)
			#print(self.localState.s)


			#publish vehicle state for crosswalk simulator
			veh_state = vehicle_state()
			veh_state.xV = self.localState.s
			veh_state.dxV = self.localState.Ux

			self.vehiclestate_pub.publish(veh_state)
			self.rate.sleep()

		#Disable inputs after test is ended
		self.enable_steer_pub.publish(0) # disable steering control.
		self.enable_acc_pub.publish(0) # disable acceleration control.



if __name__=="__main__":
	print 'Starting Controller.'
	try:
		lk = LanekeepingPublisher()
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
