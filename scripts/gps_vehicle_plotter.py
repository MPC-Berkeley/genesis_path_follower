#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from gps_utils import ref_gps_traj as r
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import display_state
from genesis_path_follower.msg import mpc_path
from plot_utils.getVehicleFrame import plotVehicle
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
import scipy.io as sio
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from lk_utils.controllers import *
from lk_utils.path_lib import *
from lk_utils.vehicle_lib import *
from lk_utils.velocityprofiles import *
from lk_utils.sim_lib import *
from matplotlib.widgets import Slider

class PlotGPSTrajectory():
	'''
	A class to plot the global GPS trajectory and the vehicle's path tracking behavior.
	Also includes the vehicle geometry, thanks to code written by Nitin Kapania, found at:
	https://github.com/nkapania/Wolverine/blob/master/utils/sim_lib.py
	'''

	
	def __init__(self):		
		

		

		# Load Global Trajectory
		if rospy.has_param('mat_waypoints'):
			mat_name = rospy.get_param('mat_waypoints')
		else:
			raise ValueError("No Matfile of waypoints were provided!")

		if not (rospy.has_param('lat0') and rospy.has_param('lon0') and rospy.has_param('yaw0')):
			raise ValueError('Invalid rosparam global origin provided!')

		lat0 = rospy.get_param('lat0')
		lon0 = rospy.get_param('lon0')
		yaw0 = rospy.get_param('yaw0')

		grt = r.GPSRefTrajectory(mat_filename=mat_name, LAT0=lat0, LON0=lon0, YAW0=yaw0) # only 1 should be valid.

		#Load road edge data
		boundFile = rospy.get_param('road_edges')
		bounds = sio.loadmat(boundFile)

		#Initialize nodes, publishers, and subscribers
		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.parseStateEstMessage, queue_size=2)
		rospy.Subscriber('display_state',display_state,self.parseDisplayStateMessage, queue_size=2)


		self.accel_pub = rospy.Publisher("/control/accel", Float32, queue_size =2)
		self.steer_pub = rospy.Publisher("/control/steer_angle", Float32, queue_size = 2)
	
		self.enable_acc_pub   = rospy.Publisher("/control/enable_accel", UInt8, queue_size =2, latch=True)  ##Why queue_size = 10?
		self.enable_steer_pub = rospy.Publisher("/control/enable_spas",  UInt8, queue_size =2, latch=True)
		self.patherrorarray=[0];
		self.Uxarray=[0];
		self.sarray=[0];
		self.velerrorarray=[0];
		self.speedProfilearray=[0];
		#self.itercount=0; #counting no. of while loop iterations in def loop
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
		self.deltapsi=0.
		self.e=0.
		self.s=0.
		self.UxDes=0.

		#Initialize vehicle
		self.genesis = Vehicle('genesis')

		
		# Set up Plots includes full ("global") trajectory, target trajectory, MPC prediction trajectory, vehicle and Speed Profile
		self.f = plt.figure()
		self.ax2=plt.gca()
		self.f2=plt.figure()
		self.ax1=plt.gca()
		self.ax = plt.gca()	
                self.f3=plt.figure()
		self.ax3=plt.gca()
		self.f4=plt.figure()
		self.ax5=plt.gca()
		
		plt.ion()
		#Create speed profile - choose between constant velocity limit or track-varying velocity limit
		self.speedProfile  = BasicProfile(self.genesis, self.path, friction = 0.4, vMax = 15., AxMax = 2.0)
		
		#self.speedProfile = BasicProfile(self.genesis, self.path, self.path.friction, self.path.vMax, AxMax = 2.0)

	
		


	
		# Set up Data
		self.edges_in = bounds["in"]
		self.edges_out = bounds["out"]

		self.x_global_traj = grt.get_Xs()
		self.y_global_traj = grt.get_Ys()
		self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		self.x_mpc_traj = self.x_global_traj[0]; self.y_mpc_traj = self.y_global_traj[0]
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = yaw0
		self.df_vehicle = 0.0	# rad	(steering angle)
		
		# This should be included in a launch file/yaml for the future.
		self.a = 1.5213  		# m  	(CoG to front axle)
		self.b = 1.4987  		#  	(CoG to rear axle)
		self.vW = 1.89	 		# m  	(vehicle width)
		self.rW = 0.3			# m		(wheel radius, estimate based on Shelley's value of 0.34 m).

		

		# Road Edges
		self.e1 = self.ax2.plot(self.edges_in[:,0], self.edges_in[:,1])
		self.e2 = self.ax2.plot(self.edges_out[:,0], self.edges_out[:,1])
		
		# Trajectory 
		self.l1, = self.ax2.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		self.l2, = self.ax2.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		self.l3, = self.ax2.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')
		
		# Vehicle coordinates
		FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire = \
			plotVehicle(self.x_vehicle, self.y_vehicle, self.psi_vehicle, self.df_vehicle, self.a, self.b, self.vW, self.rW)
			
		self.vl0, = self.ax2.plot(self.x_vehicle,      self.y_vehicle,      'bo',   MarkerSize= 8)
		self.vl1, = self.ax2.plot(FrontBody[0,:],      FrontBody[1,:],      'gray', LineWidth = 2.5)
		self.vl2, = self.ax2.plot(RearBody[0,:],       RearBody[1,:],       'gray', LineWidth = 2.5) 
		self.vl3, = self.ax2.plot(FrontAxle[0,:],      FrontAxle[1,:],      'gray', LineWidth = 2.5)
		self.vl4, = self.ax2.plot(RearAxle[0,:],       RearAxle[1,:],       'gray', LineWidth = 2.5)
		self.vl5, = self.ax2.plot(RightFrontTire[0,:], RightFrontTire[1,:], 'r',    LineWidth = 3)
		self.vl6, = self.ax2.plot(RightRearTire[0,:],  RightRearTire[1,:],  'k',    LineWidth = 3)
		self.vl7, = self.ax2.plot(LeftFrontTire[0,:],  LeftFrontTire[1,:],  'r',    LineWidth = 3)
		self.vl8, = self.ax2.plot(LeftRearTire[0,:],   LeftRearTire[1,:],   'k',    LineWidth = 3)
		
		self.ax2.set_xlabel('X (m)'); self.ax2.set_ylabel('Y (m)',fontsize=20)
		
		
		# Zoomed Inset Plot: Based off tutorial/code here: http://akuederle.com/matplotlib-zoomed-up-inset
		self.ax2_zoom = zoomed_inset_axes(self.ax2, 5, loc=2) # axis, zoom_factor, location (2 = upper left)
		self.window = 25 # m
		
		self.ze1 =  self.ax2_zoom.plot(self.edges_in[:,0], self.edges_in[:,1], 'k--')
		self.ze2 =  self.ax2_zoom.plot(self.edges_out[:,0], self.edges_out[:,1], 'k--')
		self.zl1, = self.ax2_zoom.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		self.zl2, = self.ax2_zoom.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		self.zl3, = self.ax2_zoom.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')
		self.zvl0,= self.ax2_zoom.plot(self.x_vehicle,      self.y_vehicle,      'bo',   MarkerSize= 8)
		self.zvl1,= self.ax2_zoom.plot(FrontBody[0,:],      FrontBody[1,:],      'gray', LineWidth = 2.5)
		self.zvl2,= self.ax2_zoom.plot(RearBody[0,:],       RearBody[1,:],       'gray', LineWidth = 2.5) 
		self.zvl3,= self.ax2_zoom.plot(FrontAxle[0,:],      FrontAxle[1,:],      'gray', LineWidth = 2.5)
		self.zvl4,= self.ax2_zoom.plot(RearAxle[0,:],       RearAxle[1,:],       'gray', LineWidth = 2.5)
		self.zvl5,= self.ax2_zoom.plot(RightFrontTire[0,:], RightFrontTire[1,:], 'r',    LineWidth = 3)
		self.zvl6,= self.ax2_zoom.plot(RightRearTire[0,:],  RightRearTire[1,:],  'k',    LineWidth = 3)
		self.zvl7,= self.ax2_zoom.plot(LeftFrontTire[0,:],  LeftFrontTire[1,:],  'r',    LineWidth = 3)
		self.zvl8,= self.ax2_zoom.plot(LeftRearTire[0,:],   LeftRearTire[1,:],   'k',    LineWidth = 3)
		self.ax2_zoom.set_xlim(self.x_vehicle - self.window, self.x_vehicle + self.window)
		self.ax2_zoom.set_ylim(self.y_vehicle - self.window, self.y_vehicle + self.window)
		plt.yticks(visible=False)
		plt.xticks(visible=False)
		

		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.update_state, queue_size=1)
		rospy.Subscriber('mpc_path', mpc_path, self.update_mpc_trajectory, queue_size=1)


		self.loop()

	def loop(self):
		# Main Plotting Loop.  Updates plot with info from subscriber callbacks.
		
		r  = rospy.Rate(10)
		#self.f2=plt.figure()
		#self.ax3=self.f2.add_subplot(211)
		self.itercount=0
		while not rospy.is_shutdown():
			# Update MPC OL + Reference Trajectories.
			self.l2.set_xdata(self.x_ref_traj); self.l2.set_ydata(self.y_ref_traj)
			self.l3.set_xdata(self.x_mpc_traj); self.l3.set_ydata(self.y_mpc_traj)
			self.zl2.set_xdata(self.x_ref_traj); self.zl2.set_ydata(self.y_ref_traj)
			self.zl3.set_xdata(self.x_mpc_traj); self.zl3.set_ydata(self.y_mpc_traj)
			
			# Update Vehicle Plot.
			FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire = \
				plotVehicle(self.x_vehicle, self.y_vehicle, self.psi_vehicle, self.df_vehicle, self.a, self.b, self.vW, self.rW)

			self.vl0.set_xdata(self.x_vehicle)
			self.vl0.set_ydata(self.y_vehicle)
			self.vl1.set_xdata(FrontBody[0,:])
			self.vl1.set_ydata(FrontBody[1,:])
			self.vl2.set_xdata(RearBody[0,:])
			self.vl2.set_ydata(RearBody[1,:])
			self.vl3.set_xdata(FrontAxle[0,:])
			self.vl3.set_ydata(FrontAxle[1,:])
			self.vl4.set_xdata(RearAxle[0,:])
			self.vl4.set_ydata(RearAxle[1,:])
			self.vl5.set_xdata(RightFrontTire[0,:])
			self.vl5.set_ydata(RightFrontTire[1,:])
			self.vl6.set_xdata(RightRearTire[0,:])
			self.vl6.set_ydata(RightRearTire[1,:])
			self.vl7.set_xdata(LeftFrontTire[0,:])
			self.vl7.set_ydata(LeftFrontTire[1,:])
			self.vl8.set_xdata(LeftRearTire[0,:])
			self.vl8.set_ydata(LeftRearTire[1,:])


			self.zvl0.set_xdata(self.x_vehicle)
			self.zvl0.set_ydata(self.y_vehicle)
			self.zvl1.set_xdata(FrontBody[0,:])
			self.zvl1.set_ydata(FrontBody[1,:])
			self.zvl2.set_xdata(RearBody[0,:])
			self.zvl2.set_ydata(RearBody[1,:])
			self.zvl3.set_xdata(FrontAxle[0,:])
			self.zvl3.set_ydata(FrontAxle[1,:])
			self.zvl4.set_xdata(RearAxle[0,:])
			self.zvl4.set_ydata(RearAxle[1,:])
			self.zvl5.set_xdata(RightFrontTire[0,:])
			self.zvl5.set_ydata(RightFrontTire[1,:])
			self.zvl6.set_xdata(RightRearTire[0,:])
			self.zvl6.set_ydata(RightRearTire[1,:])
			self.zvl7.set_xdata(LeftFrontTire[0,:])
			self.zvl7.set_ydata(LeftFrontTire[1,:])
			self.zvl8.set_xdata(LeftRearTire[0,:])
			self.zvl8.set_ydata(LeftRearTire[1,:])
			self.ax2_zoom.set_xlim(self.x_vehicle - self.window, self.x_vehicle + self.window)
			self.ax2_zoom.set_ylim(self.y_vehicle - self.window, self.y_vehicle + self.window)			
			
			self.f.canvas.draw()
			plt.pause(0.00001)
			#print("Error is " + str(self.e) )
			
			#if len(self.errorarray)==100:
				#self.ax3.clear()
			#	self.errorarray=[0]
			
			self.sarray.append(self.s)
			if len(self.sarray)==30:
				del self.sarray[0]
			self.Uxarray.append(self.Ux)
			if len(self.Uxarray)==30:
				del self.Uxarray[0]
			self.temp=self.speedProfile.Ux
			self.speedProfilearray.append(np.asscalar(self.speedProfile.Ux[self.itercount]))
			self.velerrorarray.append(self.speedProfilearray[len(self.speedProfilearray)-1]-self.Ux)
			
			if len(self.velerrorarray)==30:
				del self.velerrorarray[0]
				self.ax1.clear()



			
			self.ax1.plot(self.sarray,self.velerrorarray,'g',label='Velocity Error')
			self.ax1.set_ylabel('Velocity Error (m/s)',fontsize=20)
			self.ax1.set_xlim(min(self.sarray),max(self.sarray))
			self.ax1.set_autoscaley_on(True)
			
			plt.show()
			#print((self.speedProfile.Ux[100]))

			
			if len(self.speedProfilearray)==30:
				del self.speedProfilearray[0]
				self.ax5.clear()
			self.ax5.plot(self.sarray,self.speedProfilearray,'r',label='Expected Speed')
			self.ax5.set_ylabel('Expected Speed (m/s)',fontsize=20)
			self.ax5.set_xlim(min(self.sarray),max(self.sarray))
			self.ax5.set_autoscaley_on(True)
		
			self.patherrorarray.append(self.e)
			if len(self.patherrorarray)==30:
				del self.patherrorarray[0]
				self.ax3.clear()
			self.ax3.plot(self.sarray,self.patherrorarray,'b',label='Path Tracking Error')
			self.ax3.set_ylabel('Path Tracking Error (m)',fontsize=20)
			self.ax3.set_xlim(min(self.sarray),max(self.sarray))
			self.ax3.set_autoscaley_on(True)
			

			plt.show()

			plt.xticks(visible=True)
			plt.yticks(visible=True)
			plt.show()
			#print(len(self.errorarray))

			if self.itercount==0:
				self.ax1.legend(loc='lower right')
				self.ax3.legend(loc='upper right')
				self.ax5.legend(loc='lower right')
				

			self.itercount=self.itercount+1
	
			r.sleep()
			


	
	def parseStateEstMessage(self, msg):
		self.X = msg.x
		self.Y = msg.y
		self.psi = msg.psi
		self.Ux = msg.v
		self.Ax = msg.a
		self.delta =  msg.df
		
		

	def parseDisplayStateMessage(self,msg):
		self.deltapsi=msg.deltapsi
		self.s=msg.s
		self.e=msg.e
		self.Ux=msg.v
	        self.UxDes=msg.vdes

		
	
	def update_state(self, msg):
		# Update vehicle's position.
		self.x_vehicle   = msg.x
		self.y_vehicle   = msg.y
		self.psi_vehicle = msg.psi
		self.df_vehicle  = msg.df
                

	def update_mpc_trajectory(self, msg):
		# Update the MPC planned (open-loop) trajectory.
		self.x_mpc_traj = msg.xs
		self.y_mpc_traj = msg.ys
		
		# Update the reference for the MPC module.
		self.x_ref_traj = msg.xr
		self.y_ref_traj = msg.yr


if __name__=='__main__':
	p = PlotGPSTrajectory()
