#!/usr/bin/env python
import rosbag
import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np
import pdb

from rosbag_synch import MessageByTimestamp

import matplotlib.animation as manim
FFMpegWriter = manim.writers['ffmpeg']

class AnimateEMPC():
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
		#.state_msgs   = MessageByTimestamp(path_actual_bag, state_est_topic_name)
		self.mpc_path_msgs = MessageByTimestamp(path_actual_bag, mpc_path_topic_name, use_header_stamp=True)
		
		# Set up the global (i.e. full track/path) field
		#self.x_global_traj = np.ravel(path_ref['x']); self.y_global_traj = np.ravel(path_ref['y'])
		# Set up the reference (i.e. input to MPC module) field
		#self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		# Set up the predicted open loop trajectory (i.e. MPC solution) field
		#self.x_mpc_traj = self.x_global_traj[0]; self.y_mpc_traj = self.y_global_traj[0]
		
		# Set up the pose of the vehicle and store the history of the path actually taken.
		#self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = 0.0
		#self.x_vehicle_hist = []; self.y_vehicle_hist = []
		self.ts = []
		self.primal_hist = []
		self.dual_hist = []
				
		# Initial Plot Setup.
		self.f = plt.figure()		
		plt.ion()
		self.lp, = plt.plot(0, 0, 'k') 			
		self.ld, = plt.plot(0, 0, 'r')			
		plt.xlabel('t (s)'); plt.ylabel('Objective')		
		#plt.axis('equal')
		plt.xlim(0, 300); plt.ylim(0, 1000)

		# Start the video once all the data has been published at least once.  Alternatively, could use t_enable from the coresponding matfile.
		st_tm = self.mpc_path_msgs.get_start_time()
		end_tm = self.mpc_path_msgs.get_end_time()
		print st_tm, end_tm

		self.anim_loop(video_out_fname, st_tm, end_tm, 1.0)

	def anim_loop(self, video_out_name, st_tm, end_tm, dt):
		writer = FFMpegWriter(fps = int(1.0/dt))
		with writer.saving(self.f, video_out_name, 100):
			for t_query in np.arange(st_tm, end_tm, dt):
									
				msg_mpc_path = self.mpc_path_msgs.get_msg_at_tquery(t_query)
				self.update_obj_fncn(msg_mpc_path, st_tm)
						
				# Update Plot with fresh data.
				self.lp.set_xdata(self.ts); self.lp.set_ydata(self.primal_hist);
				self.ld.set_xdata(self.ts); self.ld.set_ydata(self.dual_hist);				
				self.f.canvas.draw()
				writer.grab_frame()
	
	def update_obj_fncn(self, msg, st_tm):
		t_ros = msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs
		self.ts.append( t_ros - st_tm)
		self.primal_hist.append(msg.primNNobj_long)
		self.dual_hist.append(msg.dualNNobj_long)