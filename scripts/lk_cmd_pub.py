#!/usr/bin/env python
import rospy  ##TODO: Cannot find rospy
from genesis_path_follower.msg import state_est
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
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

		self.accel_pub = rospy.Publisher("control/accel", Float32, queue_size =2)
		self.steer_pub = rospy.Publisher("control/steer_angle", Float32, queue_size = 2)

		self.enable_acc_pub   = rospy.Publisher("control/enable_accel", UInt8, queue_size =2, latch=True)  ##Why queue_size = 10?
		self.enable_steer_pub = rospy.Publisher("control/enable_spas",  UInt8, queue_size =2, latch=True)

		self.r = rospy.Rate(50.0)  ##TODO: Can we run this fast?


		#vehicle information needed - initialize to none
		self.X = None 
		self.Y = None
		self.psi = None
		self.Ux = None
		self.Ax = None
		self.delta = None

		#Initialize vehicle
		self.genesis = Vehicle('genesis')

		#Initialize Path object
		self.path = Path()
		self.path.loadFromMAT("paths/rightTurnRFSdownshifted.mat")
		self.path.setFriction(0.3)

		#Create speed profile
		self.speedProfile = velocityprofile_lib.VelocityProfile("racing")
		self.speedProfile.generate(genesis, path)

		#Create controller object - use lanekeeping
		self.controller = LaneKeepingController(path, genesis, speedProfile)

		#Create local and global states and control input object
		self.localState   = LocalState()
		self.globalState  = GlobalState()
		self.controlInput = ControlInput()

		#Initialize map matching object - use closest style
		mapMatch = MapMatch(path, "closest")

		#Enable steering
		self.enable_steer_pub.publish(1) # enable steering control.
		self.enable_acc_pub.publish(2) # enable acceleration control.

		#Start testing!
		t_start = rospy.Time.now()
		print('Path Tracking Test: %s, Started at %f' % (mode, t_start.secs + t_start.nsecs*1e-9))
		self.pub_loop()


	def parseStateEstMessage(self, msg):
		self.X = msg.X
		self.Y = msg.Y
		self.psi = msg.psi
		self.Ux = msg.vel
		self.Ax = msg.acc_filt
		self.delta =  msg.df


	def pub_loop(self):
		while not rospy.is_shutdown():
			t_now = rospy.Time.now()

			dt = t_now - t_start
			dt_secs = dt.secs + 1e-9 * dt.nsecs


			#Yaw rate and Uy currently not returned by state publisher!
			self.localState.Update(Ux = self.Ux)
			self.globalState.Update(posE = self.X, posN = self.Y, psi = self.psi)

			#Localize Vehicle
			self.mapMatch.localize(self.localState, self.globalState)

			#Calculate control inputs
			self.controller.updateInput(self.localState, self.controlInput)
			delta = self.controlInput.delta
			Fx = self.controlInput.Fx

			# use F = m*a to get desired acceleration
			accel = Fx / genesis.m

			print('Steer Command: %f deg \t Accel Command: %f ms2 \t Time: %f seconds' % (controlInput.delta, accel, dt_secs))

			#Publish control inputs

			self.steer_pub.publish(delta)
			self.accel_pub.publish(accel)

			r.sleep()

		#Disable inputs after test is ended
		enable_steer_pub.publish(0) # disable steering control.
		enabl_acc_pub.publish(0) # disable acceleration control.



if __name__=="__main__":
	print 'Starting Controller.'
	try:
		lk = LanekeepingPublisher()
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
