#!/usr/bin/env python
import rospy  ##TODO: Cannot find rospy
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import prediction
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
import sys
import pickle
import time


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
		self.prediction_pub = rospy.Publisher('OL_predictions', prediction, queue_size=1)



		self.rateHz = 10.0
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

		plt.plot(self.speedProfile.s, self.speedProfile.Ux)
		plt.show()

		#Create controller object - use lanekeeping
		self.controller = LaneKeepingController(self.path, self.genesis, self.speedProfile)

		#Create local and global states and control input object
		self.localState   = LocalState()
		self.globalState  = GlobalState(self.path)
		self.controlInput = ControlInput()
		self.OL_predictions = prediction()



		self.oldS = 0.
		self.lapCounter = 0


		self.closedLoopData = ClosedLoopData(dt = 1.0 / self.rateHz, Time = 400., v0 = 8.0)
		
		#Initialization Parameters for LMPC controller; 
		numSS_Points = 30; numSS_it = 2; N = 14
		Qslack = 100*np.diag([10.,.1, 1., 0.1, 10., 1.]); Qlane  = np.array([15. , 10.]); Q = np.zeros((6,6))
		R = 0*np.zeros((2,2)); dR = 20.0*np.array([2.5, 40.]); 
		dt = 1.0 / self.rateHz; Laps = 10; TimeLMPC = 400
		Solver = "OSQP"; steeringDelay = 0; idDelay= 0; aConstr = np.array([self.accelMin, self.accelMax]) #min and max acceleration
		
		SysID_Solver = "CVX" 
		self.halfWidth = rospy.get_param('half_width') #meters - hardcoded for now, can be property of map
		self.LMPC  = ControllerLMPC(numSS_Points, numSS_it, N, Qslack, Qlane, Q, R, dR, dt, self.path, Laps, TimeLMPC, Solver, SysID_Solver, steeringDelay, idDelay, aConstr, self.trackLength, self.halfWidth) 
		self.timeCounter = 0
		self.OneStepPredicted=self.LMPC.xPred
		self.OneStepPredictionError=self.LMPC.xPred
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
		Path_Keeping_Data_Flag=0
		Path_Keeping_Laps=2
		while not rospy.is_shutdown():
			t_now = rospy.Time.now()
			dt = t_now - t_start
			dt_secs = dt.secs + 1e-9 * dt.nsecs

			#Populate localState and globalState objects from state publisher
			self.localState.update(Ux = self.Ux, Uy = self.Uy, r = self.r)
			self.globalState.update(posE = self.X, posN = self.Y, psi = self.psi)
			self.mapMatch.localize(self.localState, self.globalState)

			xMeasuredLoc = np.array([self.localState.Ux, self.localState.Uy, self.localState.r, self.localState.deltaPsi, self.localState.s, self.localState.e])
			xMeasuredGlob  = np.array([self.localState.Ux, self.localState.Uy, self.localState.r, self.globalState.psi, self.globalState.posE, self.globalState.posN])
			if (self.OneStepPredicted!=[]):
				self.OneStepPredictionError=xMeasuredLoc-self.OneStepPredicted

			#Localize Vehicle
			
			#print("Lateral Error is " + str(self.localState.e) )
			
			#print "current state is: ", xMeasuredLoc
			#check lap counter to see if lap elapsed
			sNow = self.localState.s
			if (self.oldS - sNow) > self.trackLength / 2:
				print(self.lapCounter)
				print(self.timeCounter)
				self.LMPC.addTrajectory(self.closedLoopData)
				if Path_Keeping_Data_Flag==0 and self.lapCounter<=Path_Keeping_Laps:
				 	file_name='/data/closedLoopData%s.obj' % self.lapCounter
				 	file_data= open(sys.path[0]+file_name, 'wb')
				 	pickle.dump(self.closedLoopData, file_data)
				 	file_data.close()
				self.closedLoopData.updateInitialConditions(xMeasuredLoc, xMeasuredGlob)
				self.lapCounter += 1
				self.timeCounter = 0

			self.oldS = sNow



			#Calculate control inputs
			if Path_Keeping_Data_Flag==0:
				if self.lapCounter <= Path_Keeping_Laps:
					self.controller.updateInput(self.localState, self.controlInput)
					delta = self.controlInput.delta
					Fx = self.controlInput.Fx

					# use F = m*a to get desired acceleration. Limit acceleration command to 2 m/s
					accel = min( Fx / self.genesis.m , self.accelMax)

				else: 
					self.LMPC.solve(xMeasuredLoc)
					delta = self.LMPC.uPred[0,0]
					accel = self.LMPC.uPred[0,1]
					self.OL_predictions.epsi = self.LMPC.xPred[:, 3]
					self.OL_predictions.s    = self.LMPC.xPred[:, 4]
					self.OL_predictions.ey   = self.LMPC.xPred[:, 5]
					self.OL_predictions.SSx       = self.LMPC.SS_glob_PointSelectedTot[4, :]
					self.OL_predictions.SSy       = self.LMPC.SS_glob_PointSelectedTot[5, :]
					self.prediction_pub.publish(self.OL_predictions)
					self.OneStepPredicted=self.LMPC.xPred[1,:]
					
					#print "Terminal Constraints Slack Variable : ", self.LMPC.slack
					#print "Lane Slack Variable : ", self.LMPC.laneSlack
					#print "One  Step Prediction Error : ", self.OneStepPredictionError
					if (self.OneStepPredictionError!=[]):
						print "One Step Prediction Errors :"
						print " Ux =", self.OneStepPredictionError[0]," Uy =", self.OneStepPredictionError[1]," r =",self.OneStepPredictionError[2]," deltaPsi =", self.OneStepPredictionError[3]," s =", self.OneStepPredictionError[4], " e =", self.OneStepPredictionError[5]

					print(self.LMPC.solverTime.total_seconds()+self.LMPC.linearizationTime.total_seconds())
					if (self.LMPC.solverTime.total_seconds() + self.LMPC.linearizationTime.total_seconds()) > 1 / self.rateHz:
						print("Error: not real time feasible")

			else:
				while self.lapCounter<=Path_Keeping_Laps:
					file_name='data/closedLoopData%s.obj' % self.lapCounter
					file_data=open(sys.path[0]+file_name,'rb')
					temp_closedLoopData= pickle.load(file_data)
					file_data.close()
					self.LMPC.addTrajectory(temp_closedLoopData)
					self.lapCounter+=1
				self.LMPC.solve(xMeasuredLoc)
				delta = self.LMPC.uPred[0,0]
				accel = self.LMPC.uPred[0,1]



			#print("Accel Desired (mps2) is " + str(accel) )

			#Save the data
			
			uApplied = np.array([delta, accel])
			solverTime = 0.
			sysIDTime = 0.
			contrTime = 0.
			measSteering = 0.0

			self.closedLoopData.addMeasurement(xMeasuredGlob, xMeasuredLoc, uApplied, solverTime, sysIDTime, contrTime, measSteering)
			self.timeCounter = self.timeCounter + 1

			if self.lapCounter >= 1:
				self.LMPC.addPoint(xMeasuredLoc, xMeasuredGlob, uApplied, self.timeCounter)


			#Publish control inputs

			self.steer_pub.publish(delta)
			self.accel_pub.publish(accel)

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
