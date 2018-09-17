#!/usr/bin/env python
import rosbag
import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np

from rosbag_synch import MessageByTimestamp
from getVehicleFrame import plotVehicle

import matplotlib.animation as manim
FFMpegWriter = manim.writers['ffmpeg']

class AnimateVehicle():
	'''
	A class to plot the global GPS trajectory and the vehicle's path tracking behavior.
	Also includes the vehicle geometry, thanks to code written by Nitin Kapania, found at:
	https://github.com/nkapania/Wolverine/blob/master/utils/sim_lib.py
	'''
	def __init__(self, act_bag_fname, ref_mat_fname, video_out_fname):		

		# Load Global Trajectory
		path_ref = sio.loadmat(ref_mat_fname)
		path_actual_bag = rosbag.Bag(act_bag_fname)

		# Check the correct topic name to use (dyn or kin rosbag?):
		state_est_topic_name = '/vehicle/state_est'
		mpc_path_topic_name  = '/vehicle/mpc_path'
		if '/vehicle/state_est_dyn' in  path_actual_bag.get_type_and_topic_info()[1].keys():
			state_est_topic_name = '/vehicle/state_est_dyn'
			mpc_path_topic_name  = '/vehicle/mpc_path_dyn'		

		# Get relevant rosbag messages out and save to an array.
		self.state_msgs   = MessageByTimestamp(path_actual_bag, state_est_topic_name)
		self.mpc_path_msgs = MessageByTimestamp(path_actual_bag, mpc_path_topic_name, use_header_stamp=False)
		
		# Set up the global (i.e. full track/path) field
		self.x_global_traj = np.ravel(path_ref['x']); self.y_global_traj = np.ravel(path_ref['y'])
		# Set up the reference (i.e. input to MPC module) field
		self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		# Set up the predicted open loop trajectory (i.e. MPC solution) field
		self.x_mpc_traj = self.x_global_traj[0]; self.y_mpc_traj = self.y_global_traj[0]
		
		# Set up the pose of the vehicle and store the history of the path actually taken.
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = 0.0
		self.x_vehicle_hist = []; self.y_vehicle_hist = []
		self.df_vehicle = 0.0	# rad	(steering angle)		
		
		# Vehicle Geometry Parameters
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
		self.l2, = self.ax.plot(self.x_ref_traj,    self.y_ref_traj, 'rx', MarkerSize=8, MarkerEdgeWidth=3)	
		self.l3, = self.ax.plot(self.x_mpc_traj,    self.y_mpc_traj, 'g*', MarkerSize=12)
		self.l4, = plt.plot(self.x_vehicle_hist, self.y_vehicle_hist, 'c', LineWidth=2)
		
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
		

		# Start the video once all the data has been published at least once.  Alternatively, could use t_enable from the coresponding matfile.
		st_tm = np.max([self.mpc_path_msgs.get_start_time(), self.state_msgs.get_start_time()])			
		end_tm = np.min([self.mpc_path_msgs.get_end_time(), self.state_msgs.get_end_time()])
		print st_tm, end_tm

		self.anim_loop(video_out_fname, st_tm, end_tm, 0.1)

	def anim_loop(self, video_out_name, st_tm, end_tm, dt):
		writer = FFMpegWriter(fps = int(1.0/dt))
		with writer.saving(self.f, video_out_name, 100):
			for t_query in np.arange(st_tm, end_tm, dt):
		
				msg_state = self.state_msgs.get_msg_at_tquery(t_query)
				self.update_state(msg_state)
			
				msg_mpc_path = self.mpc_path_msgs.get_msg_at_tquery(t_query)
				self.update_mpc_trajectory(msg_mpc_path)
						
				# Update Plot with fresh data.				
				self.l2.set_xdata(self.x_ref_traj); self.l2.set_ydata(self.y_ref_traj)
				self.l3.set_xdata(self.x_mpc_traj); self.l3.set_ydata(self.y_mpc_traj)								
				self.l4.set_xdata(self.x_vehicle_hist); self.l4.set_ydata(self.y_vehicle_hist)

				self.update_vehicle_body()

				# Adjust the plot to focus on the vehicle and not show the full track.
				plt_buffer = 20.0
				plt.xlim(self.x_vehicle - plt_buffer, self.x_vehicle + plt_buffer)
				plt.ylim(self.y_vehicle - plt_buffer, self.y_vehicle + plt_buffer)

				self.f.canvas.draw()
				writer.grab_frame()

	def update_state(self, msg):
		# Update vehicle's position.
		self.x_vehicle   = msg.x
		self.y_vehicle   = msg.y
		self.psi_vehicle = msg.psi
		self.df_vehicle  = msg.df

		self.x_vehicle_hist.append(msg.x)
		self.y_vehicle_hist.append(msg.y)

	def update_mpc_trajectory(self, msg):
		# Update the MPC planned (open-loop) trajectory.
		self.x_mpc_traj = msg.xs
		self.y_mpc_traj = msg.ys
		
		# Update the reference for the MPC module.
		self.x_ref_traj = msg.xr
		self.y_ref_traj = msg.yr

	def update_vehicle_body(self):
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