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
from lk_utils.LMPC import ControllerLMPC
from lk_utils.Utilities import *
from lk_utils.Classes import LMPCprediction, ClosedLoopData
import matplotlib.pyplot as plt
import numpy as np
import math



#################################################################################################
### Integrates Ugo Rosolia's LMPC onto Genesis. First runs 3 laps of data using Nitin
### Kapania's lanekeeping controller, than after lap 3 begins to LMPC. 
#################################################################################################

class LanekeepingPublisher():

	'''
	Publishes acceleration and steering commands based on LMPC
	'''

	def __init__(self):

		#Initialize nodes, publishers, and subscribers
		rospy.init_node('lk_cmd_pub', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.parseStateEstMessage, queue_size=2)

		self.accel_pub = rospy.Publisher("/control/accel", Float32, queue_size =2)
		self.steer_pub = rospy.Publisher("/control/steer_angle", Float32, queue_size = 2)

		self.enable_acc_pub   = rospy.Publisher("/control/enable_accel", UInt8, queue_size =2, latch=True)  ##Why queue_size = 10?
		self.enable_steer_pub = rospy.Publisher("/control/enable_spas",  UInt8, queue_size =2, latch=True)

		rate = 10.0
		self.r = rospy.Rate(rate)  ##TODO: Can we run this fast?

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

		#Initialize vehicle
		self.genesis = Vehicle('genesis')

		#Create speed profile - choose between constant velocity limit or track-varying velocity limit
		self.speedProfile  = BasicProfile(self.genesis, self.path, friction = 0.4, vMax = 15., AxMax = 2.0)

		plt.plot(self.speedProfile.s, self.speedProfile.Ux)
		plt.show()

		#Create controller object - use lanekeeping
		self.controller = LaneKeepingController(self.path, self.genesis, self.speedProfile)

		#Create local and global states and control input object
		self.localState   = LocalState()
		self.globalState  = GlobalState(self.path)
		self.controlInput = ControlInput()

		self.oldS = 0.
		self.lapCounter = 0


		self.closedLoopData = ClosedLoopData(dt = 1.0 / rate, Time = 400., v0 = 8.0)
		
		#Initialization Parameters for LMPC controller; 
		numSS_Points = 15; numSS_it = 2; N = 12
		Qslack = np.diag([1., 1., 1., 1., 1., 1.]); Qlane  = np.array([15. , 10.]); Q = np.zeros((6,6))
		R = np.zeros((2,2)); dR = np.array([10., 10.]); 
		dt = 1.0 / rate; Laps = 10; TimeLMPC = 400
		Solver = "OSQP"; steeringDelay = 0.; idDelay= 0.; aConstr = np.array([2., 2.]) #min and max acceleration
		
		SysID_Solver = "CVX" 
		halfWidth = 3.0 #meters - hardcoded for now, can be property of map
		self.LMPC  = ControllerLMPC(numSS_Points, numSS_it, N, Qslack, Qlane, Q, R, dR, dt, self.path, Laps, TimeLMPC, Solver, SysID_Solver, steeringDelay, idDelay, aConstr, self.trackLength, halfWidth) 
		self.timeCounter = 0

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
		self.Uy = msg.vy #switching from Borrelli's notation to Hedrick's
		self.Ux = msg.vx #switching from Borrelli's notation to Hedrick's
		self.r = msg.wz  #switching from Borrelli's notation to Hedrick's

	def pub_loop(self):
		#Start testing!
		t_start = rospy.Time.now()
		print('Path Tracking Test: Started at %f' % (t_start.secs + t_start.nsecs*1e-9))

		while not rospy.is_shutdown():
			t_now = rospy.Time.now()
			dt = t_now - t_start
			dt_secs = dt.secs + 1e-9 * dt.nsecs

			#Populate localState and globalState objects from state publisher
			self.localState.update(Ux = self.Ux, Uy = self.Uy, r = self.r)
			self.globalState.update(posE = self.X, posN = self.Y, psi = self.psi)

			xMeasuredLoc = np.array([self.localState.Ux, self.localState.Uy, self.localState.r, self.localState.deltaPsi, self.localState.s, self.localState.e])
			xMeasuredGlob  = np.array([self.localState.Ux, self.localState.Uy, self.localState.r, self.globalState.psi, self.globalState.posE, self.globalState.posN])


			#Localize Vehicle
			self.mapMatch.localize(self.localState, self.globalState)
			#print("Lateral Error is " + str(self.localState.e) )
			
			#check lap counter to see if lap elapsed
			sNow = self.localState.s
			if (self.oldS - sNow) > self.trackLength / 2:
				self.lapCounter += 1
				self.LMPC.addTrajectory(closedLoopData)
				self.closedLoopData.updateInitialConditions(xMeasuredLoc, xMeasuredGlob)



			#Calculate control inputs
			if self.lapCounter <= 3:
				self.controller.updateInput(self.localState, self.controlInput)
				delta = self.controlInput.delta
				Fx = self.controlInput.Fx

				# use F = m*a to get desired acceleration. Limit acceleration command to 2 m/s
				accel = min( Fx / self.genesis.m , 2.0)

			else: 
				self.LMPC.solve(xMeasuredLoc)
				delta = self.LMPC.uPred[0,0]
				accel = self.LMPC.uPred[0,1]
				



			print("Accel Desired (mps2) is " + str(accel) )

			#Save the data
			
			uApplied = np.array([delta, accel])
			solverTime = 0.
			sysIDTime = 0.
			contrTime = 0.
			measSteering = 0.0


			self.closedLoopData.addMeasurement(xMeasuredGlob, xMeasuredLoc, uApplied, solverTime, sysIDTime, contrTime, measSteering)
			self.LMPC.addPoint(xMeasuredLoc, xMeasuredGlob, uApplied, self.timeCounter)
			self.timeCounter = self.timeCounter + 1






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
