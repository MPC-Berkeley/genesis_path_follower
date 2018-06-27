#!/usr/bin/env python
import rosbag
import scipy.io as sio
import numpy as np
import matplotlib.pyplot as plt
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import mpc_path
import argparse
import matplotlib.animation as manim
FFMpegWriter = manim.writers['ffmpeg']
import pdb

class MessageByTimestamp():
	def __init__(self, bag_object, topic_name, use_header_stamp=True):
		if use_header_stamp:
			self.bag_list = [(msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs, msg) \
				for _, msg, _ in bag_object.read_messages(topics=topic_name)]
		else:
			self.bag_list = [(t.secs + 1e-9*t.nsecs, msg) \
				for _, msg, t in bag_object.read_messages(topics=topic_name)]

		self.ts = np.array([x[0] for x in self.bag_list])		

	def get_start_time(self):
		return self.ts[0]

	def get_end_time(self):
		return self.ts[-1]

	def get_msg_at_tquery(self, t_query):
		if t_query < self.ts[0]:
			raise ValueError('Time stamp is earlier than collected data!')
		elif t_query > self.ts[-1]:
			raise ValueError('Time stamp is after than collected data!')

		return self.bag_list[ np.argmin(np.fabs(self.ts - t_query)) ][1]

class AnimateGPSTrajectory():
	'''
	A class to animate the global GPS trajectory and the vehicle's path tracking behavior after data collection.
	'''
	def __init__(self, act_bag_fname, ref_mat_fname, video_out_fname):		

		# Load Global Trajectory
		path_ref = sio.loadmat(ref_mat_fname)
		path_actual_bag = rosbag.Bag(act_bag_fname)

		# Get relevant rosbag messages out and save to an array.
		self.state_msgs = MessageByTimestamp(path_actual_bag, '/vehicle/state_est')
		#self.ref_path_msgs = MessageByTimestamp(path_actual_bag, '/vehicle/target_path')
		#self.mpc_path_msgs = MessageByTimestamp(path_actual_bag, '/vehicle/mpc_path')
		
		# Set up Data
		self.x_global_traj = np.ravel(path_ref['x'])
		self.y_global_traj = np.ravel(path_ref['y'])
		#self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		#self.x_mpc_traj = self.x_global_traj[0]; self.y_mpc_traj = self.y_global_traj[0]
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = 0.0
		self.x_vehicle_hist = []; self.y_vehicle_hist = []

		# Set up Plot: includes full ("global") trajectory, target trajectory, MPC prediction trajectory, and vehicle position.
		self.f = plt.figure()		
		plt.ion()
		l1, = plt.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		#l2, = plt.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		l3, = plt.plot(self.x_vehicle, self.y_vehicle, 'bo')	
		#l4, = plt.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')
		l5, = plt.plot(self.x_vehicle_hist, self.y_vehicle_hist, 'c')

		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		#self.l_arr = [l1,l2,l3,l4]
		self.l_arr = [l1, None, l3, None, l5]
		plt.axis('equal')

		print self.state_msgs.get_start_time(), self.state_msgs.get_end_time()
		# Todo: make this the latest of all messages to start and the earliest of all messages to end.
		self.anim_loop(video_out_fname, self.state_msgs.get_start_time(), self.state_msgs.get_end_time(), 0.1)

	def anim_loop(self, video_out_name, st_tm, end_tm, dt):
		writer = FFMpegWriter(fps = int(1.0/dt))
		with writer.saving(self.f, video_out_name, 100):
			for t_query in np.arange(st_tm, end_tm, dt):
				msg_state = self.state_msgs.get_msg_at_tquery(t_query)
				self.update_state(msg_state)

				'''
				msg_ref_path = self.ref_path_msgs.get_msg_at_tquery(t_query)
				self.update_ref_trajectory(msg_ref_path)
				msg_mpc_path = self.mpc_path_msgs.get_msg_at_tquery(t_query)
				self.update_mpc_trajectory(msg_mpc_path)
				'''
						
				# Update Plot with fresh data.
				#self.l_arr[1].set_xdata(self.x_ref_traj); self.l_arr[1].set_ydata(self.y_ref_traj)		
				self.l_arr[2].set_xdata(self.x_vehicle);  self.l_arr[2].set_ydata(self.y_vehicle)		
				#self.l_arr[3].set_xdata(self.x_mpc_traj); self.l_arr[3].set_ydata(self.y_mpc_traj)		
				self.l_arr[4].set_xdata(self.x_vehicle_hist); self.l_arr[4].set_ydata(self.y_vehicle_hist)
				self.f.canvas.draw()
				writer.grab_frame()

	def update_state(self, msg):
		self.x_vehicle = msg.x
		self.y_vehicle = msg.y
		self.x_vehicle_hist.append(msg.x)
		self.y_vehicle_hist.append(msg.y)

	'''
	def update_ref_trajectory(self, msg):
		self.x_ref_traj = msg.xs
		self.y_ref_traj = msg.ys

	def update_mpc_trajectory(self, msg):
		self.x_mpc_traj = msg.xs
		self.y_mpc_traj = msg.ys
	'''
if __name__=='__main__':
	parser = argparse.ArgumentParser('Convert bag file and reference mat file to a video.')
	parser.add_argument('--bag', type=str, required=True, help='Bag file to animate.')
	parser.add_argument('--mat', type=str, required=True, help='Matfile with the reference path.')
	parser.add_argument('--vid', type=str, required=True, help='Video output filename (mp4).')
	args = parser.parse_args()
	anim = AnimateGPSTrajectory(args.bag, args.mat, args.vid)