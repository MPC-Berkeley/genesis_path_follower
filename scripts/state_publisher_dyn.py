#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from genesis_msgs.msg import SteeringReport
import math as m
from genesis_path_follower.msg import state_est_dyn
from genesis_msgs.msg import VehicleImuReport
from tf.transformations import euler_from_quaternion
import numpy as np

# Vehicle State Publisher for the Hyundai Genesis.  Uses OxTS and vehicle CAN messages to localize.

''' Global Variables for Callbacks '''
tm_gps = None; lat = None; lon = None			# GPS

tm_vel = None; vel = None; acc_filt = None;		# Velocity/Acceleration
v_x = None; v_y = None;

tm_imu = None; psi = None						# Heading
tm_df  = None;  df = None						# Steering Angle (delta_f)

tm_v_imu = None; a_lat = None; a_lon = None; yaw_rate = None; # Vehicle IMU information.

def time_valid(ros_tm, tm_arr):
	# This function is meant to ensure that data is fresh relative to current time (tm_now)
	# Unused as of now.
	tm_now = ros_tm.secs + 1e-9*ros_tm.nsecs
	for tm_rcvd in tm_arr:
		diff = m.fabs(tm_now - tm_rcvd)
		if diff > 0.05: # seconds
			return False
	return True

''' GPS -> XY '''
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
	delta_lat = m.radians(lat1 - lat0)
	delta_lon = m.radians(lon1 - lon0)

	lat_avg = 0.5 * ( m.radians(lat1) + m.radians(lat0) )
	X = R_earth * delta_lon * m.cos(lat_avg)
	Y = R_earth * delta_lat

	return X,Y

''' Topic Parsing Functions '''
def parse_gps_fix(msg):
	global tm_gps, lat, lon
	tm_gps = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	lat = msg.latitude
	lon = msg.longitude

def parse_gps_vel(msg):
	global tm_vel, vel, acc_filt, psi, v_x, v_y

	v_east = msg.twist.twist.linear.x
	v_north = msg.twist.twist.linear.y
	v_gps = m.sqrt(v_east**2 + v_north**2)

	if psi == None:
		return

	v_x = np.cos(psi)  * v_east + np.sin(psi) * v_north
	v_y = -np.sin(psi) * v_east + np.cos(psi) * v_north
		

	if tm_vel is None:
		tm_vel = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
		acc_filt = 0.0

	else:
		dtm_vel = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs - tm_vel
		tm_vel = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
		acc_raw = (v_gps - vel)/dtm_vel
		acc_filt = 0.01 * acc_raw + 0.99 * acc_filt # Low Pass Filter for Acceleration

	vel = v_gps

def parse_imu_data(msg):
	# Get yaw angle.
	global tm_imu, psi

	tm_imu = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	ori = msg.orientation
	quat = (ori.x, ori.y, ori.z, ori.w)
	roll, pitch, yaw = euler_from_quaternion(quat)

	assert(-m.pi <= yaw)
	assert(m.pi >= yaw)

	psi = yaw + 0.5 * m.pi
	if psi > m.pi:
		psi = - (2*m.pi - psi)		

	# yaw in the Genesis OxTS coord system is wrt N = 0 (longitudinal axis of vehicle).
	# in the OxTS driver code, there is a minus sign for heading
	# (https://github.com/MPC-Car/GenesisAutoware/blob/master/ros/src/sensing/drivers/oxford_gps_eth/src/node.cpp#L10)
	# so heading is actually ccw radians from N = 0.

def parse_steering_angle(msg):
	# Get steering angle (wheel angle/steering ratio).
	global tm_df, df
	tm_df= msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	df = m.radians(msg.steering_wheel_angle) / 15.87

def parse_veh_imu_data(msg):
	# Get steering angle (wheel angle/steering ratio).
	global tm_v_imu, a_lat, a_lon, yaw_rate
	tm_v_imu= msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
	a_lat = msg.lat_accel
	a_lon = msg.long_accel
	yaw_rate = m.radians(msg.yaw_rate)

def pub_loop():
	rospy.init_node('state_publisher', anonymous=True)	
	rospy.Subscriber('/gps/fix', NavSatFix, parse_gps_fix, queue_size=1)
	rospy.Subscriber('/gps/vel', TwistWithCovarianceStamped, parse_gps_vel, queue_size=1)
	rospy.Subscriber('/imu/data', Imu, parse_imu_data, queue_size=1)
	rospy.Subscriber('/vehicle/steering', SteeringReport, parse_steering_angle, queue_size=1)
	rospy.Subscriber('/vehicle/imu', VehicleImuReport, parse_veh_imu_data, queue_size=1)

	if not (rospy.has_param('lat0') and rospy.has_param('lon0') and rospy.has_param('yaw0')):
		raise ValueError('Invalid rosparam global origin provided!')

	if not rospy.has_param('time_check_on'):
		raise ValueError('Did not specify if time validity should be checked!')

	LAT0 = rospy.get_param('lat0')
	LON0 = rospy.get_param('lon0')
	YAW0 = rospy.get_param('yaw0')
	time_check_on = rospy.get_param('time_check_on')
	
	state_pub = rospy.Publisher('state_est_dyn', state_est_dyn, queue_size=1)

	r = rospy.Rate(100)
	while not rospy.is_shutdown():		
		
		if None in (lat, lon, psi, vel, acc_filt, df, v_x, v_y, yaw_rate, a_lat, a_lon): 
			rospy.logfatal("Waiting for topics in state_publisher_dyn.py!")
			r.sleep() # If the vehicle state info has not been received.
			continue

		curr_state = state_est_dyn()
		curr_state.header.stamp = rospy.Time.now()
		
		# TODO: time validity check, only publish if data is fresh
		#if time_check_on and not time_valid(curr_state.header.stamp,[tm_vel, tm_df, tm_imu, tm_gps]):
		#	r.sleep()
		#	continue

		curr_state.lat = lat
		curr_state.lon = lon

		X,Y = latlon_to_XY(LAT0, LON0, lat, lon)

		curr_state.x   = X
		curr_state.y   = Y
		curr_state.psi = psi
		curr_state.v   = vel
		curr_state.vx  = v_x
		curr_state.vy  = v_y
		curr_state.wz  = yaw_rate
		
		curr_state.a   = acc_filt
		curr_state.a_lat = a_lat
		curr_state.a_lon = a_lon
		curr_state.df  = df

		state_pub.publish(curr_state)
		r.sleep()

if __name__=='__main__':
	try:
		pub_loop()
	except rospy.ROSInterruptException:
		pass
