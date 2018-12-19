#!/usr/bin/env python
import rospy
import numpy as np
from utils import *
from harpagornis.msg import action
from harpagornis.msg import cur_state
from harpagornis.msg import plotting_state
import matplotlib.pyplot as plt

#Simulates the crosswalk-pedestrian interaction problem - supports running Sarah Thornton's
#policyServer.jl 

class CrosswalkSimulator():

	def __init__(self):
		rospy.init_node('crosswalk_simulator', anonymous = True)
		rospy.Subscriber('action',action, self.parseVehicleAction, queue_size = 2)
		self.state_pub = rospy.Publisher('cur_state',cur_state, queue_size = 1)
		self.plot_pub  = rospy.Publisher('plotting_state',plotting_state,queue_size = 1)

		self.rate = rospy.get_param('rate')
		self.r = rospy.Rate(self.rate)

		#road parameters
		self.roadFraction = rospy.get_param('roadFraction') #defines position of crosswalk
		self.roadLength   = rospy.get_param('roadLength')   #meters
		self.v0           = rospy.get_param('v0')           #m/s, desired speed of vehicle
		self.gapMu        = rospy.get_param('gapMu')            #s, mean accepted gap for an adult, from feliciani et al. , in seconds
		self.gapVariance  = rospy.get_param('gapVariance')      #s, variance in accepted gap, from feliciani et al. 
		self.dt           = rospy.get_param('dt')
		self.pedStart     = rospy.get_param('pedStart')
		self.acceptedGap  = np.sqrt(self.gapVariance) * np.random.randn() + self.gapMu  #uncertainty in gap acceptance of pedestrian from Feliciani et al.
		self.numDelayedSamples= rospy.get_param('actuatorDelay') #number of timesteps
		self.carLane      = rospy.get_param('carLane')

		if self.numDelayedSamples == 0:
			pass
		else:
			self.delayedInputVector = np.zeros(self.numDelayedSamples)

		#initialize objects
		self.road = Road(length = self.roadLength)
		self.crosswalk = Crosswalk(self.road, roadFraction = self.roadFraction) 	
		self.sidewalk  = Sidewalk(self.road) 
		self.veh = Vehicle(crosswalk = self.crosswalk, sidewalk = self.sidewalk, road = self.road, v0 = self.v0, lane = self.carLane)
		self.ped = Pedestrian(self.crosswalk, self.sidewalk, acceptedGap = self.acceptedGap, start = self.pedStart)

		#initialize topic variables
		self.prevAx_mps2 = 0. #initialize prior accel to 0 to 0
		self.velocity_mps = self.v0
		self.distance_m = self.crosswalk.start[1]
		self.inCrosswalk_bool = False
		self.posture_int = 1 #1 - Distracted, 2: Stopped, 3: Walking


		#initialize simulation parameters
		self.xV = 0. #ego vehicle position, meters
		self.dxV = self.v0 #ego vehicle velocity, meters / second

		self.xFV = self.acceptedGap * self.v0 #follower vehicle position, meters
		self.dxFV = self.v0 #follower vehicle velocity, meters per second

		self.xP = self.ped.xP0 #pedestrian position, meters
		self.dxP = self.ped.v0 #pedestrian velocity, meters / second
		self.numTrials = 1
		self.time = 0.0

		#initialize acceleration to send to vehicle
		self.currAx_mps2 = 0. #commanded accel
		self.actualAccel = 0. #actual acceleration
		self.pedestrianAccel = 0. #mps2. Keep track of pedestrian acceleration as well. 

		#plt.show()
		self.pub_loop()


	def parseVehicleAction(self, msg):
		self.prevAx_mps2 = self.currAx_mps2
		self.currAx_mps2 = msg.action

	def simulateOneStep(self):
		self.pedestrianAccel,_,_ = self.ped.getAccel(self.xP, self.dxP, self.xV, self.dxV, 0, self.crosswalk)

		#comment this line out to replace with julia server policy
		self.currAx_mps2,_,_     = self.veh.getAccel(self.xP, self.dxP, self.xV, self.dxV, 0, self.pedStart)

		commandedAccel = self.currAx_mps2
		#append commanded accel to delayed input vector

		if self.numDelayedSamples == 0:
			self.actualAccel = commandedAccel
		else:
			#take the last input from the delayed input vector
			self.actualAccel = self.delayedInputVector[-1]

			#shift the delayed input vector by one
			self.delayedInputVector = np.roll(self.delayedInputVector, 1)

			#append to the delayed input vector
			self.delayedInputVector[0] = commandedAccel

		#update simulation states with euler integration
		self.dxV += self.dt * self.actualAccel
		self.dxP += self.dt * self.pedestrianAccel

		if self.dxV < 0:
			self.dxV = 0 #car cannot go backward

		self.xV += self.dt * self.dxV
		self.xP += self.dt * self.dxP
		self.xFV += self.dt * self.dxFV

		#update variables to publish to policy server. Note that prevAccel is updated in subscriber callback function
		self.velocity_mps = self.dxV #velocity of vehicle in mps
		self.distance_m = self.crosswalk.start[1] - self.xV
		self.inCrosswalk_bool = abs(self.dxP) > 0 and not self.veh.checkTransition(self.xP, self.xV, self.pedStart) #uses the same logic as FB/FFW controller
		self.time += self.dt

		#reset ICs and gap once test is completed
		if self.xV > self.roadLength:
			self.xV = 0. #vehicle position, meters
			self.dxV = self.v0 #vehicle velocity, meters
			self.xP = self.ped.xP0 #pedestrian position, meters
			self.dxP = self.ped.v0 #pedestrian velocity, meters
			self.time = 0
			self.ped.resetState()
			self.veh.resetState()
			self.delayedInputVector = np.zeros(self.numDelayedSamples)

			#update gap and number of trials for next time
			self.acceptedGap  = np.sqrt(self.gapVariance) * np.random.randn() + self.gapMu
			self.xFV = self.acceptedGap * self.v0  
			self.numTrials += 1
			print('Starting Trial %d, with gap %02f seconds' % (self.numTrials, self.acceptedGap))
			self.ped.updateGap(self.acceptedGap)

	def pub_loop(self):
		#Start testing!
		t_start = rospy.Time.now()
		print('Crosswalk Test: Trial Number %d' % (self.numTrials))

		while not rospy.is_shutdown():
			t_now = rospy.Time.now()
			dt = t_now - t_start
			dt_secs = dt.secs + 1e-9 * dt.nsecs

			current_state = cur_state()
			current_state.header.stamp = rospy.Time.now()

			plot_state = plotting_state()
			plot_state.header.stamp = rospy.Time.now()

			self.simulateOneStep()

			current_state.velocity_mps = self.velocity_mps
			current_state.distance_m = self.distance_m
			current_state.inCrosswalk_bool = self.inCrosswalk_bool
			current_state.posture_int = self.posture_int
			current_state.prevAx_mps2 = self.prevAx_mps2

			plot_state.xP = self.xP
			plot_state.xV = self.xV
			plot_state.xFV = self.xFV
			plot_state.ddxV = self.currAx_mps2
			plot_state.time = self.time
			plot_state.trialNumber = self.numTrials
			plot_state.acceptedGap = self.acceptedGap

			self.state_pub.publish(current_state)
			self.plot_pub.publish(plot_state)
			self.r.sleep()


if __name__=="__main__":
	print 'Starting Simulation.'
	try:
		cs = CrosswalkSimulator()
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly





















# def pub_loop():
# 	rospy.init_node('state_publisher', anonymous=True)	

# 	r = rospy.Rate(100)
# 	while not rospy.is_shutdown():		
		
# 		curr_state = from_raptor()
# 		curr_state.header.stamp = rospy.Time.now()
# 		curr_state.velocity_mps = 10.
# 		curr_state.distance_m = 20.
# 		curr_state.inCrosswalk_bool = False
# 		curr_state.posture_int = 1
# 		curr_state.prevAx_mps2 = 2.0 
		
		
# 		state_pub.publish(curr_state)
# 		r.sleep()

# if __name__=='__main__':
# 	try:
# 		pub_loop()
# 	except rospy.ROSInterruptException:
# 		pass

