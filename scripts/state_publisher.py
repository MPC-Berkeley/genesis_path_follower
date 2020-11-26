#!/usr/bin/env python
import rospy
import math as m
import numpy as np
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from genesis_msgs.msg import SteeringReport
from genesis_path_follower.msg import state_est

# Vehicle State Publisher for the Hyundai Genesis.  Uses OxTS and vehicle CAN messages to localize.
# You will need to add the package from https://github.com/MPC-Berkeley/genesis_msgs for the steering angle measurement.

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

''' Take ROS timestamp in seconds from a message '''
def extract_ros_time(msg):
	return msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

''' Check if timestamps are within thresh seconds of current time '''
def is_time_valid(tm_now, tm_arr, thresh=0.05):
	for tm_rcvd in tm_arr:
		diff = m.fabs(tm_now - tm_rcvd)
		if diff > thresh: # seconds
			return False
	return True

''' Main Class to Publish Vehicle State Information '''
class StatePublisher(object):
	def __init__(self):
		rospy.init_node('state_publisher', anonymous=True)	

		attrs = ['tm_gps', 'lat', 'lon', 'x', 'y', \
		         'tm_vel', 'v', 'v_long', 'v_lat', \
		         'tm_imu', 'psi', 'long_accel', 'lat_accel', 'yaw_rate' \
		         'tm_df', 'df']
		for attr in attrs:
			setattr(self, attr, None)
		self.tm_df  = None
		self.tm_gps = None
		self.tm_vel = None
		self.tm_imu = None

		if not (rospy.has_param('lat0') and rospy.has_param('lon0')):
			raise ValueError('Invalid rosparam global origin provided!')

		if not rospy.has_param('time_check_on'):
			raise ValueError('Did not specify if time validity should be checked!')

		self.LAT0 = rospy.get_param('lat0')
		self.LON0 = rospy.get_param('lon0')	
		time_check_on = rospy.get_param('time_check_on')
		
		state_pub = rospy.Publisher('state_est', state_est, queue_size=1)

		rospy.Subscriber('/gps/fix', NavSatFix, self._parse_gps_fix, queue_size=1)
		rospy.Subscriber('/gps/vel', TwistWithCovarianceStamped, self._parse_gps_vel, queue_size=1)
		rospy.Subscriber('/imu/data', Imu, self._parse_imu_data, queue_size=1)
		rospy.Subscriber('/vehicle/steering', SteeringReport, self._parse_steering_angle, queue_size=1)

		r = rospy.Rate(100)
		while not rospy.is_shutdown():		
			
			if None in [self.tm_gps, self.tm_vel, self.tm_imu, self.tm_df]: 
				r.sleep() # If the vehicle state info has not been received.
				continue

			curr_state = state_est()
			curr_state.header.stamp = rospy.Time.now()
			
			if time_check_on:
			    time_valid = is_time_valid(extract_ros_time(curr_state), \
			    	                      [self.tm_gps, self.tm_vel, self.tm_imu, self.tm_df])
			    if not time_valid:
					r.sleep()
					continue

			curr_state.lat = self.lat
			curr_state.lon = self.lon

			curr_state.x   = self.x
			curr_state.y   = self.y
			curr_state.psi = self.psi
			curr_state.v   = self.v

			curr_state.v_long   = self.v_long
			curr_state.v_lat    = self.v_lat
			curr_state.yaw_rate = self.yaw_rate
			
			curr_state.a_long = self.long_accel
			curr_state.a_lat  = self.lat_accel
			curr_state.df     = self.df

			state_pub.publish(curr_state)
			r.sleep()

	def _parse_gps_fix(self, msg):
		# This function gets the latitude and longitude from the OxTS and does localization.		
		self.lat    = msg.latitude
		self.lon    = msg.longitude
		self.x, self.y = latlon_to_XY(self.LAT0, self.LON0, self.lat, self.lon)

		self.tm_gps = extract_ros_time(msg)

	def _parse_gps_vel(self, msg):
		# This function gets the velocity information from the OXTS.
		# It rotates the East/North velocity into the body frame.

		if not self.psi:
			return # Wait until we get the orientation first.

		psi = self.psi # Could set a lock here.  This is copy by value so should be okay.

		v_east = msg.twist.twist.linear.x
		v_north = msg.twist.twist.linear.y
				
		self.v      =  m.sqrt(v_east**2 + v_north**2)             # speed (m/s)
		self.v_long =  m.cos(psi) * v_east + m.sin(psi) * v_north # longitudinal velocity (m/s)
		self.v_lat  = -m.sin(psi) * v_east + m.cos(psi) * v_north # lateral velocity (m/s)

		self.tm_vel =  extract_ros_time(msg)

	def _parse_imu_data(self, msg):
		# This function gets the yaw angle and acceleration/yaw rate from the OXTS.		

		# Get yaw angle from quaternion representation.
		ori = msg.orientation
		quat = (ori.x, ori.y, ori.z, ori.w)
		_, _, yaw = euler_from_quaternion(quat)

		# The yaw/heading given directly by the OxTS is measured counterclockwise from N.
		# That's why we offset by 0.5 * pi and don't need to change the sign.
		psi = yaw + 0.5 * m.pi
		psi = (psi + np.pi) % (2. * np.pi) - np.pi # wrap within [-pi, pi]
		self.psi = psi # yaw (rad), measured counterclockwise from E (global x-axis).

		# Get acceleration and yaw rate information.  The y-axis of the OXTS points right.
		self.long_accel =  msg.linear_acceleration.x # m/s^2
		self.lat_accel  = -msg.linear_acceleration.y # m/s^2
		self.yaw_rate   =  msg.angular_velocity.z    # rad/s

		self.tm_imu = extract_ros_time(msg)

	def _parse_steering_angle(self, msg):
		# Get steering angle (steering wheel angle/steering ratio).		
		self.df = m.radians(msg.steering_wheel_angle) / 15.87 # steering angle, delta_f (rad)

		self.tm_df = extract_ros_time(msg)


if __name__=='__main__':
	try:
		StatePublisher()
	except rospy.ROSInterruptException:
		pass
