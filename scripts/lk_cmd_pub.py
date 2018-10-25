#!/usr/bin/env python
import rospy  ##TODO: Cannot find rospy
from genesis_path_follower.msg import state_est
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from lk_utils.controllers import *
from lk_utils.path_lib import *
from lk_utils.vehicle_lib import *
from lk_utils.velocityprofiles import *
from lk_utils.sim_lib import *
import matplotlib.pyplot as plt

import math



#################################################################################################
### Path following code using lanekeeping controller from VSD 2015 paper ########################
#################################################################################################

class LanekeepingPublisher():

	'''
	Publishes acceleration and steering commands based on Nitin Kapania's thesis. 
	@Nitin Kapania Aug 2018
	'''

	def __init__(self):

		#Initialize nodes, publishers, and subscribers
		rospy.init_node('lk_cmd_pub', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.parseStateEstMessage, queue_size=2)

		self.accel_pub = rospy.Publisher("/control/accel", Float32, queue_size =2)
		self.steer_pub = rospy.Publisher("/control/steer_angle", Float32, queue_size = 2)
		self.displaystate_pub = rospy.Publisher('display_state', state_est, queue_size=1)

		self.enable_acc_pub   = rospy.Publisher("/control/enable_accel", UInt8, queue_size =2, latch=True)  ##Why queue_size = 10?
		self.enable_steer_pub = rospy.Publisher("/control/enable_spas",  UInt8, queue_size =2, latch=True)

		self.r = rospy.Rate(50.0)  ##TODO: Can we run this fast?

		#Initialize Path object
		self.path = Path()
		if not rospy.has_param('mat_waypoints'):
			raise ValueError('Invalid rosparam global origin provided!')	

		pathLocation = rospy.get_param('mat_waypoints')	
		self.path.loadFromMAT(pathLocation)

		#vehicle information needed - initialize to none
		self.X = self.path.posE[0] 
		self.Y = self.path.posN[0] 
		self.psi = self.path.roadPsi[0]
		self.Ux = 0.
		self.Ax = 0.
		self.delta = 0.
		self.e=0.
		self.deltapsi=0.
		self.s=0.		
		#Initialize vehicle
		self.genesis = Vehicle('genesis')

		#Create speed profile - choose between constant velocity limit or track-varying velocity limit
		self.speedProfile  = BasicProfile(self.genesis, self.path, friction = 0.4, vMax = 15., AxMax = 2.0)
		#self.speedProfile = BasicProfile(self.genesis, self.path, self.path.friction, self.path.vMax, AxMax = 2.0)

		plt.plot(self.speedProfile.s, self.speedProfile.Ux)
		plt.show()

		#Create controller object - use lanekeeping
		self.controller = LaneKeepingController(self.path, self.genesis, self.speedProfile)

		#Create local and global states and control input object
		self.localState   = LocalState()
		self.globalState  = GlobalState(self.path)
		self.controlInput = ControlInput()

		#Initialize map matching object - use closest style
		self.mapMatch = MapMatch(self.path, "closest")
		

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

	def pub_loop(self):
		#Start testing!
		t_start = rospy.Time.now()
		print('Path Tracking Test: Started at %f' % (t_start.secs + t_start.nsecs*1e-9))

		while not rospy.is_shutdown():
			t_now = rospy.Time.now()
			dt = t_now - t_start
			dt_secs = dt.secs + 1e-9 * dt.nsecs

			#Yaw rate and Uy currently not returned by state publisher!
			self.localState.update(Ux = self.Ux)
			self.globalState.update(posE = self.X, posN = self.Y, psi = self.psi)

			#Localize Vehicle
			self.mapMatch.localize(self.localState, self.globalState)
			#print("Lateral Error is " + str(self.localState.e) )
			

			#Calculate control inputs
			self.controller.updateInput(self.localState, self.controlInput)
			delta = self.controlInput.delta
			Fx = self.controlInput.Fx

			# use F = m*a to get desired acceleration. Limit acceleration command to 2 m/s
			accel = min( Fx / self.genesis.m , 2.0)
			print("Accel Desired (mps2) is " + str(accel) )
			
			#if None in (deltapsi,s,e): 

			#r.sleep() 
			# If the vehicle state info has not been received.
			#continue

		curr_state = display_state()
		curr_state.header.stamp = rospy.Time.now()
		
		# TODO: time validity check, only publish if data is fresh
		#if time_check_on and not time_valid(curr_state.header.stamp,[tm_vel, tm_df, tm_imu, tm_gps]):
		#	r.sleep()
		#	continue

		#curr_state.deltapsi = self.localState.deltapsi
		curr_state.e=self.localState.e
		#curr_state.s=self.localState.s
		
	
		displaystate_pub.publish(curr_state)


		#Publish control inputs

		self.steer_pub.publish(delta)
		self.accel_pub.publish(accel)
		self.r.sleep()

		#Disable inputs after test is ended
		self.enable_steer_pub.publish(0) # disable steering control.
		self.enable_acc_pub.publish(0) # disable acceleration control.



if __name__=="__main__":
	print 'Starting Controller.'
	try:
		lk = LanekeepingPublisher()
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
