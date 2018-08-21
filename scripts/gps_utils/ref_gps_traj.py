#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math
import rosbag
import time
import rospy
import scipy.io as sio

################################################
# HELPER FUNCTIONS
################################################
def latlon_to_XY(lat0, lon0, lat1, lon1):
	''' 
	Convert latitude and longitude to global X, Y coordinates,
	using an equirectangular projection.

	X = meters east of lon0
	Y = meters north of lat0

	Sources: http://www.movable-type.co.uk/scripts/latlong.html
		     https://github.com/MPC-Car/StochasticLC/blob/master/controller.py
	'''
	R_earth = 6371000 # meters
	delta_lat = math.radians(lat1 - lat0)
	delta_lon = math.radians(lon1 - lon0)

	lat_avg = 0.5 * ( math.radians(lat1) + math.radians(lat0) )
	X = R_earth * delta_lon * math.cos(lat_avg)
	Y = R_earth * delta_lat

	return X,Y

################################################
# GPSRefTrajectory Class
################################################
class GPSRefTrajectory():
	'''
	Class to load a matfile GPS trajectory and provide functions to obtain a
	local trajectory from the full trajectory for MPC using the current vehicle state.
	'''	
	def __init__(self, mat_filename=None, LAT0=None, LON0=None, YAW0=None, traj_horizon=8, traj_dt=0.2):
		if type(LAT0) != float or type(LON0) != float or type(YAW0) != float:
			raise ValueError('Did not provide proper origin info.')

		if mat_filename is None:
			raise ValueError('Invalid matfile specified.')
		
		self.traj_horizon = traj_horizon	# horizon (# time steps ahead) for trajectory reference
		self.traj_dt = traj_dt				# time discretization (s) for each time step in horizon

		tms  = []		# ROS Timestamps (s)
		lats = []		# Latitude (decimal degrees)
		lons = []		# Longitude (decimal degrees)
		yaws = []		# heading (radians ccw wrt East, i.e.global X-axis, aka psi)
		Xs   = []		# global X position (m, wrt to origin at LON0, LAT0)
		Ys   = []		# global Y position (m, wrt to origin at LON0, LAT0)
		cdists = []		# cumulative distance along path (m, aka "s" in Frenet formulation)

		data_dict = sio.loadmat(mat_filename)
			
		tms  = np.ravel(data_dict['t'])
		lats = np.ravel(data_dict['lat'])
		lons = np.ravel(data_dict['lon'])
		yaws = np.ravel(data_dict['psi'])
		cdists = np.ravel(data_dict['cdists'])
		curv = data_dict['curv']	# this is already a good matrix; size 6626 x 4 (polyDeg+1), no need to ravel
		vels = np.ravel(data_dict['v'])

		for i in range(len(lats)):
			lat = lats[i]; lon = lons[i]
			X,Y = latlon_to_XY(LAT0, LON0, lat, lon) # Greg's magic function
			# yaw already provided

			# cdists already provided by path-file; cdists = "s" ; unit of "s" is m
			# if len(Xs) == 0: 		# i.e. the first point on the trajectory
				# cdists.append(0.0)	# s = 0
			# else:					# later points on the trajectory
				# d = math.sqrt( (X - Xs[-1])**2 + (Y - Ys[-1])**2 ) + cdists[-1]
				# cdists.append(d) 	# s = s_prev + dist(z[i], z[i-1])
			Xs.append(X)
			Ys.append(Y)

		# global trajectory matrix
		self.trajectory =  np.column_stack((tms, lats, lons, yaws, Xs, Ys, cdists, vels, curv)) 
		

		# interpolated path or what I call "local trajectory" -> reference to MPC
		self.x_interp	= None
		self.y_interp	= None
		self.psi_interp	= None
		self.curv_interp = None	# added curvature for FRENET
		self.s_interp = None	# added, not sure needed
		self.v_interp = None
		
		# some handles for plotting (see plot_interpolation)
		self.f = None
		self.l_arr = None

	def get_global_trajectory_reference(self):
		return self.trajectory

	def get_Xs(self):
		return self.trajectory[:,4]

	def get_Ys(self):
		return self.trajectory[:,5]

	def get_yaws(self):
		return self.trajectory[:,3]
		
	# Main callback function to get the waypoints from the vehicle's initial pose and the prerecorded global trajectory.
	def get_waypoints(self, X_init, Y_init, yaw_init, v_target=None):

		XY_traj = self.trajectory[:,4:6]		# full XY global trajectory
		xy_query = np.array([[X_init,Y_init]])	# the vehicle's current position (XY)

		# find the index of the closest point on the trajectory to the initial vehicle pose
		diff_dists = np.sum( (XY_traj-xy_query)**2, axis=1)
		closest_traj_ind = np.argmin(diff_dists) 

		# TODO: this function does not handle well the case where the car is far from the recorded path!  Ill-defined behavior/speed.
		# May use np.min(diff_dists) and add appropriate logic to handle this edge case.

		if v_target is not None:
			return self.__waypoints_using_vtarget(closest_traj_ind, v_target, yaw_init) #v_ref given, use distance information for interpolation
		else:
			return self.__waypoints_using_time(closest_traj_ind, yaw_init)			  #no v_ref, use time information for interpolation

	####### FRENET ######
	# s_ref, K_coeff, stop_cmd, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref = grt[:get_waypoints_frenet](x_curr, y_curr, psi_curr)
	def get_waypoints_frenet(self, X_init, Y_init, yaw_init, v_target=None):
		XY_traj = self.trajectory[:,4:6]	# excluding 6 (python notation)
		xy_query = np.array([[X_init,Y_init]])	# the vehicle's current position (X,Y); size: 1 by 2

		# find the index of the closest point on the trajectory to the initial vehicle pose
		diff_dists = np.sum( (XY_traj - xy_query)**2, axis=1 )
		closest_traj_ind = np.argmin(diff_dists) 	# extract index along path
		ey_abs_sq = np.min(diff_dists)		# compute ey^2
		ey_abs = np.sqrt(ey_abs_sq)			# |ey| (sign not known, computed later on)
		if v_target is not None:
			print('not properly implemented yet')
			return self.__waypoints_using_time_frenet(closest_traj_ind, yaw_init, ey_abs, X_init, Y_init)			  #NOT PROPERLY IMPLEMENTED - JUST SO THAT IT WORKS

			# return self.__waypoints_using_vtarget(closest_traj_ind, v_target, yaw_init) #v_ref given, use distance information for interpolation
		else:  # currently used
			# return self.__waypoints_using_time(closest_traj_ind, yaw_init)			  #no v_ref, use time information for interpolation
			return self.__waypoints_using_time_frenet(closest_traj_ind, yaw_init, ey_abs, X_init, Y_init)			  #no v_ref, use time information for interpolation



	# Visualization Function to plot the vehicle's current position, the full global trajectory, and the local trajectory for MPC.
	def plot_interpolation(self, x,y):
		if self.x_interp is None:
			print 'First call get_waypoints before plotting!'
			return

		if self.f is None:
		# figure creation		
			self.f = plt.figure()
			plt.ion()

			l1, = plt.plot(self.trajectory[:,4], self.trajectory[:,5], 'k') # global trajectory
			l2, = plt.plot(self.x_interp, self.y_interp, 'rx')			  	# local trajectory using vehicle's current position
			l3, = plt.plot(x,y, 'bo')									   	# vehicle's current position
			self.l_arr = [l1,l2,l3]
			plt.axis('equal')

		else:
		# figure update
			self.l_arr[1].set_xdata(self.x_interp)
			self.l_arr[1].set_ydata(self.y_interp)
			self.l_arr[2].set_xdata(x)
			self.l_arr[2].set_ydata(y)

		self.f.canvas.draw()
		plt.pause(0.05)	

	''' Helper functions: you shouldn't need to call these! '''
	# hence it starts with __FUNCTIONNAME 
	def __waypoints_using_vtarget(self, closest_traj_ind, v_target, yaw_init):
		start_dist = self.trajectory[closest_traj_ind,6] # cumulative dist corresponding to look ahead point

		dists_to_fit = [x*self.traj_dt*v_target + start_dist for x in range(0,self.traj_horizon+1)] # edit bounds
		# NOTE: np.interp returns the first value x[0] if t < t[0] and the last value x[-1] if t > t[-1].
		self.x_interp = np.interp(dists_to_fit, self.trajectory[:,6], self.trajectory[:,4]) 	 # x_des = f_interp(d_des, d_actual, x_actual)
		self.y_interp = np.interp(dists_to_fit, self.trajectory[:,6], self.trajectory[:,5]) 	 # y_des = f_interp(d_des, d_actual, y_actual)
		psi_ref = np.interp(dists_to_fit, self.trajectory[:,6], self.trajectory[:,3])    # psi_des = f_interp(d_des, d_actual, psi_actual)
		self.psi_interp = self.__fix_heading_wraparound(psi_ref, yaw_init)

		# Send a stop command if the end of the trajectory is within the horizon of the waypoints.
		# Alternatively, could use start_dist as well: if start_dist + some delta_s > end_dist, then stop.
		stop_cmd = False
		if self.x_interp[-1] == self.trajectory[-1,4] and self.y_interp[-1] == self.trajectory[-1,5]:
			stop_cmd = True

		return self.x_interp, self.y_interp, self.psi_interp, stop_cmd

	def __waypoints_using_time(self, closest_traj_ind, yaw_init):
		start_tm = self.trajectory[closest_traj_ind,0] # t0, time of recorded trajectory corresponding to closest point
		
		times_to_fit = [h*self.traj_dt + start_tm for h in range(0,self.traj_horizon+1)]
		# NOTE: np.interp returns the first value x[0] if t < t[0] and the last value x[-1] if t > t[-1].
		self.x_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,4]) 	 # x_des = f_interp(t_des, t_actual, x_actual)
		self.y_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,5]) 	 # y_des = f_interp(t_des, t_actual, y_actual)
		psi_ref = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,3])    # psi_des = f_interp(t_des, t_actual, psi_actual)
		self.psi_interp = self.__fix_heading_wraparound(psi_ref, yaw_init)

		# Send a stop command if the end of the trajectory is within the horizon of the waypoints.
		# Alternatively, could use start_dist as well: if start_dist + some delta_s > end_dist, then stop.
		stop_cmd = False
		if self.x_interp[-1] == self.trajectory[-1,4] and self.y_interp[-1] == self.trajectory[-1,5]:
			stop_cmd = True

		return self.x_interp, self.y_interp, self.psi_interp, stop_cmd


	#### FRENET ###
	# return: (s_ref, K_coeff, stop_cmd, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref, v_ref); (x_ref, y_ref, psi_ref) only for plotting
	# arguments: (closest_traj_ind, yaw_init, ey_abs, X_init, Y_init)
	def __waypoints_using_time_frenet(self, closest_traj_ind, yaw_init, ey_abs, X_init, Y_init):
		# 	self.trajectory =  np.column_stack((tms, lats, lons, yaws, Xs, Ys, cdists, curv)) 
		start_tm = self.trajectory[closest_traj_ind,0]	# corresponds to look ahead point OR CURRENT ???????

		# extract corresponding time indices
		times_to_fit = [h*self.traj_dt + start_tm for h in range(0,self.traj_horizon+1)]

		# get (X,Y, Psi) for plotting purposes
		# NOTE: np.interp returns the first value x[0] if t < t[0] and the last value x[-1] if t > t[-1].
		self.x_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,4]) 	 # x_des = f_interp(t_des, t_actual, x_actual)
		self.y_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,5]) 	 # y_des = f_interp(t_des, t_actual, y_actual)
		self.v_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,7])


		psi_ref = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,3])    # psi_des = f_interp(t_des, t_actual, psi_actual)
		self.psi_interp = self.__fix_heading_wraparound(psi_ref, yaw_init)

		self.s_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,6])	# compute the look-ahead s
		self.curv_interp = self.trajectory[closest_traj_ind,8:]	# extract local c(s) polynomial
		stop_cmd = False
		if self.s_interp[-1] == self.trajectory[-1,6]:
			stop_cmd = True

		# determine s_curr, epsi_curr
		s_curr = self.trajectory[closest_traj_ind, 6]	# determine current progress along s
		epsi_curr = yaw_init - self.trajectory[closest_traj_ind,3]	# e_psi = psi - \theta(s), where psi is vehicle-yaw and theta(s) is curve-yaw
		
		# VG EDITS
		s_curr_vg = self.s_interp[0]
		epsi_curr_vg = yaw_init - self.psi_interp[0]

		# better approximation of ey
		lat_error_X = X_init - self.x_interp[0]
		lat_error_Y = Y_init - self.y_interp[0]
		frenet_x = lat_error_X * np.cos(self.psi_interp[0]) + lat_error_Y * np.sin(self.psi_interp[0])
		frenet_y = -lat_error_X * np.sin(self.psi_interp[0]) + lat_error_Y * np.cos(self.psi_interp[0])
		ey_curr_vg = frenet_y


		# to compute e_y, follow Rajamani's book, equations (2.54), (2.55)
		# X = X_des - e_y * sin(psi), Y = Y_des + e_y*cos(psi)
		# seems correct
		X_des = self.trajectory[closest_traj_ind,4]
		Y_des = self.trajectory[closest_traj_ind,5]
		cos_psi = np.cos(yaw_init)
		sin_psi = np.sin(yaw_init)
		if abs(cos_psi) >= abs(sin_psi): 	# division is more stable
			ey = (Y_init - Y_des)/cos_psi	# seems correct
		else:
			ey = (X_des - X_init)/sin_psi	# seems correct
		ey_curr = np.sign(ey)*ey_abs	# use ey_abs b/c numerically more stable; ey only used to determine sign


		# print(ey_curr)
		# print(ey_curr_vg)

		return self.s_interp, self.curv_interp, stop_cmd, s_curr, ey_curr_vg, epsi_curr, self.x_interp, self.y_interp, self.psi_interp, self.v_interp


	def __fix_heading_wraparound(self, psi_ref, psi_current):
		# This code ensures that the psi_reference agrees with psi_current, that there are no jumps by +/- 2*pi.
		# This logic is based on the fact for a given angle on the unit circle, every other angle is within +/- pi away.
		check_1 = np.max(np.fabs(np.diff(psi_ref))) < np.pi
		check_2 = np.max(np.fabs(psi_ref - psi_current)) < np.pi

		if check_1 and check_2:
			return psi_ref

		for i in range(len(psi_ref)):
			# pick the reference by adding +/- 2*pi s.t. psi_ref is close to psi_current = no jumps.
			p = psi_ref[i]
			psi_cands_arr = np.array([p, p + 2*np.pi, p - 2*np.pi])
			best_cand = np.argmin(np.fabs(psi_cands_arr - psi_current))

			psi_ref[i] = psi_cands_arr[best_cand]

		return psi_ref
