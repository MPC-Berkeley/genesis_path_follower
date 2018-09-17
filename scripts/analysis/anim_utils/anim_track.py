#!/usr/bin/env python
import rosbag
import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np

from rosbag_synch import MessageByTimestamp

import matplotlib.animation as manim
FFMpegWriter = manim.writers['ffmpeg']

class AnimateTrack():
	'''
	A class to animate the global GPS trajectory and the vehicle's position on the track.
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


		# Initial Plot Setup.
		self.f = plt.figure()		
		plt.ion()
		l1, = plt.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		l2, = plt.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		l3, = plt.plot(self.x_vehicle, self.y_vehicle, 'bo')	
		l4, = plt.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')
		l5, = plt.plot(self.x_vehicle_hist, self.y_vehicle_hist, 'c')

		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		self.l_arr = [l1,l2,l3,l4,l5]		
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
				self.l_arr[1].set_xdata(self.x_ref_traj); self.l_arr[1].set_ydata(self.y_ref_traj)						
				self.l_arr[3].set_xdata(self.x_mpc_traj); self.l_arr[3].set_ydata(self.y_mpc_traj)		
				self.l_arr[2].set_xdata(self.x_vehicle);  self.l_arr[2].set_ydata(self.y_vehicle)		
				self.l_arr[4].set_xdata(self.x_vehicle_hist); self.l_arr[4].set_ydata(self.y_vehicle_hist)
				self.f.canvas.draw()

				writer.grab_frame()

	def update_state(self, msg):
		self.x_vehicle = msg.x
		self.y_vehicle = msg.y
		self.x_vehicle_hist.append(msg.x)
		self.y_vehicle_hist.append(msg.y)
	
	def update_mpc_trajectory(self, msg):
		self.x_mpc_traj = msg.xs
		self.y_mpc_traj = msg.ys
		self.x_ref_traj = msg.xr
		self.y_ref_traj = msg.yr