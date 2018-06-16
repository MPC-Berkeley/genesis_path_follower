#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import ref_gps_traj as r
import rospy
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import mpc_path
import pdb

class PlotGPSTrajectory():
	'''
	A class to plot the global GPS trajectory and the vehicle's path tracking behavior.
	'''
	def __init__(self):		
		csv_name = None
		pkl_name = None
		if rospy.has_param('csv_waypoints'):
			csv_name = rospy.get_param('csv_waypoints')
		elif rospy.has_param('pkl_waypoints'):
			pkl_name = rospy.get_param('pkl_waypoints')
		else:
			raise ValueError("No CSV/PKL of waypoints were provided!")
		
		grt = r.GPSRefTrajectory(csv_filename=csv_name, pkl_filename = pkl_name) # only 1 should be valid.
		
		x_global_traj = grt.get_Xs()
		y_global_traj = grt.get_Ys()

		self.f = plt.figure()		
		plt.ion()
		l1, = plt.plot(x_global_traj, y_global_traj, 'k') 			# global trajectory
		l2, = plt.plot(x_global_traj[0], y_global_traj[0], 'rx')	# local trajectory using vehicle's current position
		l3, = plt.plot(x_global_traj[0], y_global_traj[0], 'bo')	# vehicle's current position
		l4, = plt.plot(x_global_traj[0], y_global_traj[0], 'g*')	# MPC open-loop trajectory to follow local trajectory
		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		self.l_arr = [l1,l2,l3,l4]
		plt.axis('equal')

		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.update_state, queue_size=1)
		rospy.Subscriber('target_path', mpc_path, self.update_local_trajectory, queue_size=1)
		rospy.Subscriber('mpc_path', mpc_path, self.update_mpc_trajectory, queue_size=1)
		self.loop()

	def loop(self):
		r  = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.f.canvas.draw()
			plt.pause(0.001)
			r.sleep()

	def update_local_trajectory(self, msg):
		self.l_arr[1].set_xdata(msg.xs); self.l_arr[1].set_ydata(msg.ys)

	def update_state(self, msg):
		self.l_arr[2].set_xdata(msg.x); self.l_arr[2].set_ydata(msg.y)

	def update_mpc_trajectory(self, msg):
		self.l_arr[3].set_xdata(msg.xs); self.l_arr[3].set_ydata(msg.ys)

if __name__=='__main__':
	p = PlotGPSTrajectory()
	
