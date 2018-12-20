#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils import *
from genesis_path_follower.msg import plotting_state
from matplotlib import animation
import pdb

'''
A class to visualize the pedestrian / vehicle interaction.  
'''


class PlotSimulationTrajectory():
	def __init__(self):

		#load road parameters
		self.roadFraction = rospy.get_param('roadFraction') #defines position of crosswalk
		self.roadLength   = rospy.get_param('roadLength')   #meters
		self.rate 		  = rospy.get_param('rate') #Hz
		self.dt           = rospy.get_param('dt') #seconds
		self.v0           = rospy.get_param('v0') #m/s
		self.pedStart     = rospy.get_param('pedStart') #start
		self.carLane      = rospy.get_param('carLane')

		#Initialize nodes, publishers, and subscribers
		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('plotting_state', plotting_state, self.parsePlottingMessage, queue_size=2)
		
		self.r = rospy.Rate(self.rate)  ##TODO: Can we run this fast?

		#Initialize objects
		self.road = Road(length = self.roadLength)
		self.crosswalk = Crosswalk(self.road, roadFraction = self.roadFraction) 	
		self.sidewalk  = Sidewalk(self.road) 
		self.veh = Vehicle(crosswalk = self.crosswalk, sidewalk = self.sidewalk, road = self.road, lane = self.carLane)
		self.ped = Pedestrian(self.crosswalk, self.sidewalk, start = self.pedStart)


		#Initialize variables
		self.xP = 0.0
		self.xV = 0.0
		self.vehicleX = self.veh.lane * self.road.laneWidth + self.road.laneWidth / 2 - self.veh.width/2 #x position of vehicle - assumed constant
		self.pedestrianY = self.crosswalk.start[1] #y position of pedestrian - assumed constant

		#Allow for interactive plotting and get axis and figure handles
		plt.ion
		self.fig = self.sidewalk.fig
		self.ax  = self.sidewalk.ax

		#initialize rectangles representing vehicle, following vehicle and pedestrian
		self.vehRect = patches.Rectangle((0,0), self.veh.width, self.veh.height, fc = 'k')
		self.pedRect = patches.Rectangle((0,0), self.ped.radius, self.ped.radius, fc = 'r')

		self.ax.add_patch(self.vehRect)
		self.ax.add_patch(self.pedRect)
		self.accel = 0.

		self.loop()


	def parsePlottingMessage(self, msg):
		self.xP    = msg.xP
		self.xV    = msg.xV
		self.accel = msg.ddxV
		self.acceptedGap = msg.acceptedGap


	def loop(self):
		while not rospy.is_shutdown():
			self.fig.canvas.draw()
			plt.title("acceleration = " + str(self.accel))


			#update position of ego vehicle
			self.vehRect.set_xy([self.vehicleX, self.xV])

			#update position of follower vehicle and pedestrian 
			# self.followVehRect.set_xy([self.vehicleX, self.xFV])
			self.pedRect.set_xy([self.xP, self.pedestrianY])

			plt.pause(.001)

			self.r.sleep()


if __name__=='__main__':
	pst = PlotSimulationTrajectory()








# fig, ax  = plt.subplots(1)
# plt.ion()

# a    = patches.Rectangle((0, 0), .25, .25, fc='k')
# ax.add_patch(a)

# for i in range(100):
# 	fig.canvas.draw()

# 	a.set_width(0.5)
# 	a.set_height(0.5)
# 	a.set_xy([0.25, .25*i/100]) #Note that xV is vertical for the car

# 	plt.pause(0.1)
