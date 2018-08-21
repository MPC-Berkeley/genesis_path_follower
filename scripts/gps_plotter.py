#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from gps_utils import ref_gps_traj as r
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import mpc_path

class PlotGPSTrajectory():
	'''
	A class to plot the global GPS trajectory and the vehicle's path tracking behavior.
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

		# Set up Data
		self.x_global_traj = grt.get_Xs()
		self.y_global_traj = grt.get_Ys()
		self.x_ref_traj = self.x_global_traj[0]; self.y_ref_traj = self.y_global_traj[0]
		self.x_mpc_traj = self.x_global_traj[0]; self.y_mpc_traj = self.y_global_traj[0]
		self.x_vehicle = self.x_global_traj[0];  self.y_vehicle = self.y_global_traj[0]; self.psi_vehicle = 0.0


		# Set up Plot: includes full ("global") trajectory, target trajectory, MPC prediction trajectory, and vehicle position.
		self.f = plt.figure()		
		plt.ion()
		l1, = plt.plot(self.x_global_traj, self.y_global_traj, 'k') 			
		l2, = plt.plot(self.x_ref_traj,    self.y_ref_traj, 'rx')	
		l3, = plt.plot(self.x_vehicle, self.y_vehicle, 'bo')	
		l4, = plt.plot(self.x_mpc_traj, self.y_mpc_traj, 'g*')

		plt.xlabel('X (m)'); plt.ylabel('Y (m)')
		self.l_arr = [l1,l2,l3,l4]
		plt.axis('equal')

		rospy.init_node('vehicle_plotter', anonymous=True)
		rospy.Subscriber('state_est', state_est, self.update_state, queue_size=1)
		rospy.Subscriber('mpc_path', mpc_path, self.update_mpc_trajectory, queue_size=1)
		self.loop()

	def loop(self):
		# Main Plotting Loop.  Updates plot with info from subscriber callbacks.
		r  = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Update Plot with fresh data.
			self.l_arr[1].set_xdata(self.x_ref_traj); self.l_arr[1].set_ydata(self.y_ref_traj)		
			self.l_arr[2].set_xdata(self.x_vehicle);  self.l_arr[2].set_ydata(self.y_vehicle)		
			self.l_arr[3].set_xdata(self.x_mpc_traj); self.l_arr[3].set_ydata(self.y_mpc_traj)		
			self.f.canvas.draw()
			plt.pause(0.001)
			r.sleep()

	def update_state(self, msg):
		# Update vehicle's position.
		self.x_vehicle = msg.x
		self.y_vehicle = msg.y

	def update_mpc_trajectory(self, msg):
		# Update the MPC planned (open-loop) trajectory.
		self.x_mpc_traj = msg.xs
		self.y_mpc_traj = msg.ys
		
		# Update the reference for the MPC module.
		self.x_ref_traj = msg.xr_recon
		self.y_ref_traj = msg.yr_recon

if __name__=='__main__':
	p = PlotGPSTrajectory()
