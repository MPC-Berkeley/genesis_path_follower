#!/usr/bin/env python
import numpy as np
import csv
import matplotlib.pyplot as plt
import math
import rosbag
import time
import rospy
import pickle as pkl

'''
RFS GLOBAL COORDINATE SYSTEM:
X axis pointing east, Y axis point north.
Centered right before stop sign near California Path Parking Lot.
'''
def heading_to_yaw(heading):
		'''
		Convert heading angle in deg (0 = N, 90 = E, 180 = S, 270 = W) into
		yaw angle in rad (0 = E, pi/2 = N, +/- pi = W, -pi/2 = S)
		'''
		yaw = 90.0 - heading

		while yaw >= 180.0:
			yaw -= 360.0

		while yaw <= -180.0:
			yaw += 360.0
		
		yaw = math.radians(yaw)
		assert(-math.pi <= yaw <= math.pi)
		
		return yaw

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

class GPSRefTrajectory():
	'''
	Class to encapsulate a GPS trajectory and provide functions to obtain a
	local trajectory from the full trajectory for MPC and to animate results.
	'''
	def __init__(self, csv_filename = None, pkl_filename = None, traj_horizon=8, traj_dt = 0.2):
		if csv_filename is None and pkl_filename is None:
			raise ValueError('A CSV or PKL file of poses should be provided!')
		elif csv_filename is not None and pkl_filename is not None:
			raise ValueError('Either a CSV or PKL file should be given, not both!')
		
		if not (rospy.has_param('lat0') and rospy.has_param('lon0') and rospy.has_param('yaw0')):
			raise ValueError('Invalid rosparam global origin provided!')

		if not rospy.has_param('is_heading_info'):
			raise ValueError('Invalid rosparam for if heading or yaw info provided!')

		LAT0 = rospy.get_param('lat0')
		LON0 = rospy.get_param('lon0')
		YAW0 = rospy.get_param('yaw0')
		use_heading = rospy.get_param('is_heading_info')
		
		# Expect CSV to contain lines with following 4 fields:
		# tm lat lon <theta/yaw>
		# (1) tm = time in seconds
		# (2) lat = latitude in decimal degrees
		# (3) lon = longitude in decimal degrees
		# (4a) theta = heading angle (N = 0, E = 90, etc.) -> if provided, this is converted to yaw
		#  OR
		# (4b) yaw (E = 0, N = 90, etc.)

		self.traj_horizon = traj_horizon	# horizon (# time steps ahead) for trajectory reference
		self.traj_dt = traj_dt				# time discretization (s) for trajectory reference

		tms  = []
		lats = []
		lons = []
		yaws = []
		Xs   = []
		Ys   = []
		cdists = []

		if csv_filename is not None:
			with open(csv_filename, 'r') as f:
				# Parse the CSV File to extract global trajectory.
				reader = csv.reader(f, delimiter=' ')
				for row in reader:
					tms.append( float(row[0]) )
					
					lat = float(row[1])
					lon = float(row[2])
					X,Y = latlon_to_XY(LAT0, LON0, lat, lon)

					lats.append( lat )
					lons.append( lon )

					if len(Xs) == 0: 		# i.e. the first point on the trajectory
						cdists.append(0.0)	# s = 0
					else:					# later points on the trajectory
						d = math.sqrt( (X - Xs[-1])**2 + (Y - Ys[-1])**2 ) + cdists[-1]
						cdists.append(d) 	# s = s_prev + dist(z[i], z[i-1])

					Xs.append(X)
					Ys.append(Y)

					if use_heading:
						yaws.append( heading_to_yaw(float(row[3])) )
					else:
						yaws.append( float(row[3]) )

		elif pkl_filename is not None:
			data_dict = pkl.load(open(pkl_filename, 'rb'))
			data_arr = data_dict['data']
			tms = data_arr[:,0]
			lats = data_arr[:,1]
			lons = data_arr[:,2]
			yaws = data_arr[:,5]

			for i in range(len(lats)):
				lat = lats[i]; lon = lons[i]
				X,Y = latlon_to_XY(LAT0, LON0, lat, lon)
				if len(Xs) == 0: 		# i.e. the first point on the trajectory
					cdists.append(0.0)	# s = 0
				else:					# later points on the trajectory
					d = math.sqrt( (X - Xs[-1])**2 + (Y - Ys[-1])**2 ) + cdists[-1]
					cdists.append(d) 	# s = s_prev + dist(z[i], z[i-1])
				Xs.append(X)
				Ys.append(Y)

		# global trajectory matrix
		self.trajectory =  np.column_stack((tms, lats, lons, yaws, Xs, Ys, cdists)) 

		
		# interpolated path or what I call "local trajectory"
		self.x_interp	= None
		self.y_interp	= None
		self.psi_interp	= None
		
		# some handles for plotting (see plot_interpolation)
		self.f = None
		self.l_arr = None

	def get_global_trajectory_reference(self):
		return self.trajectory

	def get_Xs(self):
		return self.trajectory[:,4]

	def get_Ys(self):
		return self.trajectory[:,5]

	def get_psis(self):
		return self.trajectory[:,3]
		
	# Main callback function to get the waypoints from the vehicle's initial pose and the prerecorded global trajectory.
	def get_waypoints(self, X_init, Y_init, yaw_init, v_target=None):
		XY_traj = self.trajectory[:,4:6]		# full XY global trajectory
		xy_query = np.array([[X_init,Y_init]])	# the vehicle's current position (XY)

		# find the index of the closest point on the trajectory to the initial vehicle pose
		diff_dists = np.sum( (XY_traj-xy_query)**2, axis=1)
		closest_traj_ind = np.argmin(diff_dists) 

		if v_target is not None:
			return self.__waypoints_using_vtarget(closest_traj_ind, v_target) #v_ref given, use distance information for interpolation
		else:
			return self.__waypoints_using_time(closest_traj_ind)			  #no v_ref, use time information for interpolation

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
	def __waypoints_using_vtarget(self, closest_traj_ind, v_target):
		start_dist = self.trajectory[closest_traj_ind,6] # cumulative dist corresponding to look ahead point

		dists_to_fit = [x*self.traj_dt*v_target + start_dist for x in range(1,self.traj_horizon+2)] # edit bounds
		# NOTE: np.interp returns the first value x[0] if t < t[0] and the last value x[-1] if t > t[-1].
		self.x_interp = np.interp(dists_to_fit, self.trajectory[:,6], self.trajectory[:,4]) 	 # x_des = f_interp(d_des, d_actual, x_actual)
		self.y_interp = np.interp(dists_to_fit, self.trajectory[:,6], self.trajectory[:,5]) 	 # y_des = f_interp(d_des, d_actual, y_actual)
		self.psi_interp = np.interp(dists_to_fit, self.trajectory[:,6], self.trajectory[:,3])    # psi_des = f_interp(d_des, d_actual, psi_actual)

		stop_cmd = False
		if self.x_interp[-1] == self.trajectory[-1,4] and self.y_interp[-1] == self.trajectory[-1,5]:
			stop_cmd = True

		return self.x_interp, self.y_interp, self.psi_interp, stop_cmd

	def __waypoints_using_time(self, closest_traj_ind):
		start_tm = self.trajectory[closest_traj_ind,0] # tm corresponding to look ahead point
		
		times_to_fit = [h*self.traj_dt + start_tm for h in range(0,self.traj_horizon+1)]
		# NOTE: np.interp returns the first value x[0] if t < t[0] and the last value x[-1] if t > t[-1].
		self.x_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,4]) 	 # x_des = f_interp(t_des, t_actual, x_actual)
		self.y_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,5]) 	 # y_des = f_interp(t_des, t_actual, y_actual)
		self.psi_interp = np.interp(times_to_fit, self.trajectory[:,0], self.trajectory[:,3])    # psi_des = f_interp(t_des, t_actual, psi_actual)

		stop_cmd = False
		if self.x_interp[-1] == self.trajectory[-1,4] and self.y_interp[-1] == self.trajectory[-1,5]:
			stop_cmd = True

		return self.x_interp, self.y_interp, self.psi_interp, stop_cmd

	def __waypoints_to_stop(self, closest_traj_ind):
		pass

'''
if __name__ == '__main__':
	# Testing of the GPSRefTrajectory Class.
	grt = GPSRefTrajectory(_filename='/home/govvijay/catkin_ws/src/genesis_path_follower/path_csvs/acc_azera_gps_waypoints.csv')	

	# For each point on the actual trajectory, add some Gaussian noise so initial tracking error > 0 and compute the local trajectory.
	# get_waypoints() finds the local/interpolated trajectory with respect to the current vehicle pose (x_fit, y_fit, yaw_fit)
	# plot_interpolation then plots the fitted results.
	for i in range(0, grt.trajectory.shape[0], 100):
		e_tm = grt.trajectory[i,0] - grt.trajectory[0,0]
		x_fit = grt.trajectory[i,4] + np.random.normal(loc=0.0, scale=1.0)
		y_fit = grt.trajectory[i,5] + np.random.normal(loc=0.0, scale=1.0)
		yaw_fit = grt.trajectory[i,3] 

		grt.get_waypoints(x_fit, y_fit, yaw_fit,v_target=100.0)		
		grt.plot_interpolation(x_fit,y_fit)
'''