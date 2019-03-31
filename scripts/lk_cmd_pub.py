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
import os


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

		rospy.on_shutdown(self.shutdownFunc)

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
		self.Uy = 0
		self.r = 0
		self.Ax = 0.
		self.delta = 0.
		# self.accelMax = 9.8
		# self.accelMin = 9.8 #negative value implied by LMPC controller
		self.accelMax = 4.0
		self.accelMin = 5.0 #negative value implied by LMPC controller

		#Initialize vehicle
		self.genesis = Vehicle('genesis')

		#Create speed profile - choose between constant velocity limit or track-varying velocity limit
		self.speedProfile  = BasicProfile(self.genesis, self.path, friction = 0.1, vMax = 8., AxMax = 2.)

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

		self.closedLoopData = ClosedLoopData(dt = 1.0 / self.rateHz, Time = 800., v0 = 8.0)
		# homedir = os.path.expanduser("~")
		# file_data = open(homedir+'/genesis_data/ClosedLoopDataLMPCexp.obj', 'rb')
		# self.ClosedLoopexp = pickle.load(file_data)
		# self.LMPCexp = pickle.load(file_data)
		# self.LMPCOpenLoopDataexp = pickle.load(file_data)
		# file_data.close()
		#Initialization Parameters for LMPC controller; 
		numSS_Points = 40; numSS_it = 2; N = 10
		Qslack  =  5 * np.diag([ 1.0, 0.1, 0.1, 0.1, 10, 1])          # Cost on the slack variable for the terminal constraint
		Qlane   =  np.array([50, 10]) # Quadratic slack lane cost

		Q = np.zeros((6,6))
		R = 0*np.zeros((2,2)); dR =  1 * np.array([ 25.0, 1.0]) # Input rate cost u 
		#R = np.array([[1.0, 0.0],[0.0, 0.0]]); dR =  1 * np.array([ 1.0, 1.0]) # Input rate cost u 
		dt = 1.0 / self.rateHz; Laps = 50; TimeLMPC = 500
		Solver = "OSQP"; steeringDelay = 0; idDelay= 0; aConstr = np.array([self.accelMin, self.accelMax]) #min and max acceleration
		
		SysID_Solver = "CVX" 
		self.halfWidth = rospy.get_param('half_width') #meters - hardcoded for now, can be property of map
		self.LMPC  = ControllerLMPC(numSS_Points, numSS_it, N, Qslack, Qlane, Q,      R,      dR,      dt, self.path, Laps, TimeLMPC, Solver,      SysID_Solver,           steeringDelay, idDelay, aConstr, self.trackLength, self.halfWidth) 
		self.openLoopData = LMPCprediction(N, 6, 2, TimeLMPC, numSS_Points, Laps)

		self.timeCounter = 0
		self.OneStepPredicted=self.LMPC.xPred
		self.OneStepPredictionError=self.LMPC.xPred
		#Initialize map matching object - use closest style
		self.mapMatch = MapMatch(self.path, "embed")
		

		#Enable steering
		self.enable_steer_pub.publish(0) # enable steering control.
		self.enable_acc_pub.publish(0) # enable acceleration control.

		self.pub_loop()

	def shutdownFunc(self):
		homedir = os.path.expanduser("~")    
		print("Start Saving Data")
		print(homedir)  
		# == Start: Save Data
		file_data = open(homedir+'/genesis_data'+'/ClosedLoopDataLMPC.obj', 'wb')
		pickle.dump(self.closedLoopData, file_data)
		print("Data Saved closedLoopData")    

		pickle.dump(self.LMPC, file_data)
		print("Data Saved LMPC")    

		pickle.dump(self.openLoopData, file_data)	        
		print("Data Saved openLoopData")    
		file_data.close()
		print("Data Saved Correctly")    
		# == End: Save Data


	def parseStateEstMessage(self, msg):
		self.X     = msg.x
		self.Y     = msg.y
		self.psi   = msg.psi
		self.Ax    = msg.a
		self.delta = msg.df/1.1715
		self.Uy    = msg.vy #msg.vy #switching from Borrelli's notation to Hedrick's
		self.Ux    = msg.vx #switching from Borrelli's notation to Hedrick's
		self.r     = msg.wz  #switching from Borrelli's notation to Hedrick's


		#print(msg.data)

	def pub_loop(self):
		#Start testing!
		t_start = rospy.Time.now()
		print('Path Tracking Test: Started at %f' % (t_start.secs + t_start.nsecs*1e-9))
		Path_Keeping_Data_Flag=0
		Path_Keeping_Laps=2
		oneStepPrediction = np.array([0, 0, 0, 0, 0, 0])
		



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
				self.closedLoopData.updateInitialConditions(xMeasuredLoc, xMeasuredGlob)
				self.lapCounter += 1
				self.timeCounter = 0
				# if self.lapCounter==4:
				# 	self.LMPC.update(self.LMPCexp.SS, self.LMPCexp.uSS, self.LMPCexp.Qfun, self.LMPCexp.TimeSS, self.lapCounter, self.LMPCexp.LinPoints, self.LMPCexp.LinInput)

			self.oldS = sNow



			#Calculate control inputs
			if self.lapCounter <= Path_Keeping_Laps:


				desiredErrorArray = np.array([0.0, 1.0, -1.0])
				desiredError = desiredErrorArray[self.lapCounter]
				self.controller.updateInput(self.localState, self.controlInput, desiredError)
				delta = self.controlInput.delta+0.08*np.sin(1.0*np.pi*rospy.get_time())
				Fx = self.controlInput.Fx+0.8*np.sin(1.0*np.pi*rospy.get_time())

				# use F = m*a to get desired acceleration. Limit acceleration command to 2 m/s
				accel = min( Fx / self.genesis.m , self.accelMax)

				self.steer_pub.publish(delta)
				self.accel_pub.publish(accel)
				measSteering=self.delta
				uApplied = np.array([delta, accel])

			else: 
				self.steer_pub.publish(delta)
				self.accel_pub.publish(accel)
				measSteering=self.delta
				uApplied = np.array([delta, accel])

				self.LMPC.OldSteering.append(delta)
				# self.LMPC.OldSteering.append(measSteering)
				self.LMPC.OldAccelera.append(accel)

				self.LMPC.OldSteering.pop(0)
				self.LMPC.OldAccelera.pop(0)    
				# print Controller.OldAccelera, Controller.OldSteering

				oneStepPredictionError = xMeasuredLoc - oneStepPrediction # Subtract the local measurement to the previously predicted one step

				uRealApplied = [self.LMPC.OldSteering[-1 - self.LMPC.steeringDelay], self.LMPC.OldAccelera[-1]]
				

				# print uAppliedDelay, Controller.OldSteering
				oneStepPrediction, oneStepPredictionTime = self.LMPC.oneStepPrediction(xMeasuredLoc, uRealApplied, 0)
			
				self.LMPC.solve(oneStepPrediction)
				# self.LMPC.A0[:,:,self.timeCounter,self.lapCounter-Path_Keeping_Laps]=self.LMPC.A[0]
				delta = self.LMPC.uPred[0 + self.LMPC.steeringDelay, 0]
				accel = self.LMPC.uPred[0, 1]

				self.OL_predictions.epsi = self.LMPC.xPred[:, 3]
				self.OL_predictions.s    = self.LMPC.xPred[:, 4]
				self.OL_predictions.ey   = self.LMPC.xPred[:, 5]
				self.OL_predictions.SSx       = self.LMPC.SS_glob_PointSelectedTot[4, :]
				self.OL_predictions.SSy       = self.LMPC.SS_glob_PointSelectedTot[5, :]
				self.prediction_pub.publish(self.OL_predictions)
				self.OneStepPredicted=self.LMPC.xPred[1,:]
				

				self.openLoopData.oneStepPredictionError[:,self.timeCounter, self.LMPC.it]      = oneStepPredictionError               
				self.openLoopData.PredictedStates[0:(self.LMPC.N+1),:,self.timeCounter, self.LMPC.it] = self.LMPC.xPred
				self.openLoopData.PredictedInputs[0:(self.LMPC.N), :, self.timeCounter, self.LMPC.it] = self.LMPC.uPred
				self.openLoopData.SSused[:, :, self.timeCounter, self.LMPC.it]                  = self.LMPC.SS_PointSelectedTot
				self.openLoopData.Qfunused[:, self.timeCounter, self.LMPC.it]                   = self.LMPC.Qfun_SelectedTot

				#print "Terminal Constraints Slack Variable : ", self.LMPC.slack
				#print "Lane Slack Variable : ", self.LMPC.laneSlack
				#print "One  Step Prediction Error : ", self.OneStepPredictionError
				# if (self.OneStepPredictionError!=[]):
				# 	print "One Step Prediction Errors :"
				# 	print " Ux =", self.OneStepPredictionError[0]," Uy =", self.OneStepPredictionError[1]," r =",self.OneStepPredictionError[2]," deltaPsi =", self.OneStepPredictionError[3]," s =", self.OneStepPredictionError[4], " e =", self.OneStepPredictionError[5]

				# print(self.LMPC.solverTime.total_seconds()+self.LMPC.linearizationTime.total_seconds())
				if (self.LMPC.solverTime.total_seconds() + self.LMPC.linearizationTime.total_seconds()) > 1 / self.rateHz:
					print("Error: not real time feasible")

			



			#print("Accel Desired (mps2) is " + str(accel) )

			#Save the data
			solverTime = self.LMPC.solverTime.total_seconds()
			sysIDTime = self.LMPC.linearizationTime.total_seconds()
			contrTime = self.LMPC.solverTime.total_seconds() + self.LMPC.linearizationTime.total_seconds()
			

			self.closedLoopData.addMeasurement(xMeasuredGlob, xMeasuredLoc, uApplied, solverTime, sysIDTime, contrTime, measSteering)

			if self.lapCounter >= 1:
				self.LMPC.addPoint(xMeasuredLoc, xMeasuredGlob, uApplied, self.timeCounter)

			self.timeCounter = self.timeCounter + 1

			self.rate.sleep()

		#Disable inputs after test is ended
		self.enable_steer_pub.publish(0) # disable steering control.
		self.enable_acc_pub.publish(0) # disable acceleration control.
		


if __name__=="__main__":
	print 'Starting Controller.'
	lk = LanekeepingPublisher()

