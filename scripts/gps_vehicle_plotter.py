#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from gps_utils import ref_gps_traj as r
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import mpc_path
from plot_utils.getVehicleFrame import plotVehicle
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
import scipy.io as sio

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

		#Load road edge data ((if it is given)
		use_road_edges = False
		if rospy.has_param('road_edges'):
			boundFile = rospy.get_param('road_edges')
			bounds = sio.loadmat(boundFile)
			self.edges_in = bounds["in"]
			self.edges_out = bounds["out"]
			use_road_edges = True

		# Set up Data
		self.x_global_traj = grt.get_Xs()
		self.y_global_traj = grt.get_Ys()
		self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		self.x_mpc_traj = self.x_global_traj[0]; self.y_mpc_traj = self.y_global_traj[0]
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = yaw0
		self.df_vehicle = 0.0	# rad	(steering angle)
		
		# This should be included in a launch file/yaml for the future.
		self.a = 1.5213  		# m  	(CoG to front axle)
		self.b = 1.4987  		# m  	(CoG to rear axle)
		self.vW = 1.89	 		# m  	(vehicle width)
		self.rW = 0.3			# m		(wheel radius, estimate based on Shelley's value of 0.34 m).

		# Set up Plot: includes full ("global") trajectory, target trajectory, MPC prediction trajectory, and vehicle.
		self.f = plt.figure()
		self.ax = plt.gca()		
		plt.ion()

		# Trajectory 
		self.l1, = self.ax.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		self.l2, = self.ax.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		self.l3, = self.ax.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')
		
		# Vehicle coordinates
		FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire = \
			plotVehicle(self.x_vehicle, self.y_vehicle, self.psi_vehicle, self.df_vehicle, self.a, self.b, self.vW, self.rW)
			
		self.vl0, = self.ax.plot(self.x_vehicle,      self.y_vehicle,      'bo',   MarkerSize= 8)
		self.vl1, = self.ax.plot(FrontBody[0,:],      FrontBody[1,:],      'gray', LineWidth = 2.5)
		self.vl2, = self.ax.plot(RearBody[0,:],       RearBody[1,:],       'gray', LineWidth = 2.5) 
		self.vl3, = self.ax.plot(FrontAxle[0,:],      FrontAxle[1,:],      'gray', LineWidth = 2.5)
		self.vl4, = self.ax.plot(RearAxle[0,:],       RearAxle[1,:],       'gray', LineWidth = 2.5)
		self.vl5, = self.ax.plot(RightFrontTire[0,:], RightFrontTire[1,:], 'r',    LineWidth = 3)
		self.vl6, = self.ax.plot(RightRearTire[0,:],  RightRearTire[1,:],  'k',    LineWidth = 3)
		self.vl7, = self.ax.plot(LeftFrontTire[0,:],  LeftFrontTire[1,:],  'r',    LineWidth = 3)
		self.vl8, = self.ax.plot(LeftRearTire[0,:],   LeftRearTire[1,:],   'k',    LineWidth = 3)
		
		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		plt.axis('equal')
		
		# Zoomed Inset Plot: Based off tutorial/code here: http://akuederle.com/matplotlib-zoomed-up-inset
		self.ax_zoom = zoomed_inset_axes(self.ax, 5, loc=2) # axis, zoom_factor, location (2 = upper left)
		self.window = 25 # m
		self.zl1, = self.ax_zoom.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		self.zl2, = self.ax_zoom.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		self.zl3, = self.ax_zoom.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')
		self.zvl0,= self.ax_zoom.plot(self.x_vehicle,      self.y_vehicle,      'bo',   MarkerSize= 8)
		self.zvl1,= self.ax_zoom.plot(FrontBody[0,:],      FrontBody[1,:],      'gray', LineWidth = 2.5)
		self.zvl2,= self.ax_zoom.plot(RearBody[0,:],       RearBody[1,:],       'gray', LineWidth = 2.5) 
		self.zvl3,= self.ax_zoom.plot(FrontAxle[0,:],      FrontAxle[1,:],      'gray', LineWidth = 2.5)
		self.zvl4,= self.ax_zoom.plot(RearAxle[0,:],       RearAxle[1,:],       'gray', LineWidth = 2.5)
		self.zvl5,= self.ax_zoom.plot(RightFrontTire[0,:], RightFrontTire[1,:], 'r',    LineWidth = 3)
		self.zvl6,= self.ax_zoom.plot(RightRearTire[0,:],  RightRearTire[1,:],  'k',    LineWidth = 3)
		self.zvl7,= self.ax_zoom.plot(LeftFrontTire[0,:],  LeftFrontTire[1,:],  'r',    LineWidth = 3)
		self.zvl8,= self.ax_zoom.plot(LeftRearTire[0,:],   LeftRearTire[1,:],   'k',    LineWidth = 3)
		self.ax_zoom.set_xlim(self.x_vehicle - self.window, self.x_vehicle + self.window)
		self.ax_zoom.set_ylim(self.y_vehicle - self.window, self.y_vehicle + self.window)
		plt.yticks(visible=False)
		plt.xticks(visible=False)

		# Road Edges (if given)
		if use_road_edges:
			self.e1, = self.ax.plot(self.edges_in[:,0], self.edges_in[:,1])
			self.e2, = self.ax.plot(self.edges_out[:,0], self.edges_out[:,1])
			self.ze1, =  self.ax_zoom.plot(self.edges_in[:,0], self.edges_in[:,1], 'k--')
			self.ze2, =  self.ax_zoom.plot(self.edges_out[:,0], self.edges_out[:,1], 'k--')
		
		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.update_state, queue_size=1)
		rospy.Subscriber('mpc_path', mpc_path, self.update_mpc_trajectory, queue_size=1)
		self.loop()

	def loop(self):
		# Main Plotting Loop.  Updates plot with info from subscriber callbacks.
		r  = rospy.Rate(10)
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
			
			self.ax_zoom.set_xlim(self.x_vehicle - self.window, self.x_vehicle + self.window)
			self.ax_zoom.set_ylim(self.y_vehicle - self.window, self.y_vehicle + self.window)
			
			self.f.canvas.draw()
			plt.pause(0.001)
			r.sleep()

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
