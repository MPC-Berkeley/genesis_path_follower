#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32 as float_msg
from genesis_path_follower.msg import state_est

class VehicleSimulator():
	'''
	A vehicle dynamics simulator using a linear tire model.  This uses the Genesis control/estimation ROS topics.
	Modified from Ugo Rosolia's Code: https://github.com/urosolia/RacingLMPC/blob/master/src/fnc/SysModel.py
	'''
	def __init__(self):
		rospy.init_node('vehicle_simulator', anonymous=True)
		rospy.Subscriber('/control/accel', float_msg, self._acc_cmd_callback, queue_size=1)
		rospy.Subscriber('/control/steer_angle', float_msg, self._df_cmd_callback, queue_size=1)
		self.state_pub = rospy.Publisher('state_est', state_est, queue_size=1)

		self.tcmd_a = None	# rostime (s) of received acc command
		self.tcmd_d = None	# rostime (s) of received df command
		self.acc = 0.0		# actual acceleration (m/s^2)
		self.df = 0.0		# actual steering angle (rad)
		self.acc_des = 0.0	# desired acceleration	(m/s^2)
		self.df_des = 0.0	# desired steering_angle (rad)

		self.dt_model = 0.01				# vehicle model update period (s) and frequency (Hz)
		self.hz = int(1.0/self.dt_model)
		self.r = rospy.Rate(self.hz)
		
		# Simulated Vehicle State.
		self.X   = rospy.get_param('X0', -300.0) 	# X position (m)
		self.Y   = rospy.get_param('Y0', -450.0) 	# Y position (m)
		self.psi = rospy.get_param('Psi0', 1.0) 	# yaw angle (rad)
		self.vx  = rospy.get_param('V0', 0.0)		# longitudinal velocity (m/s)
		self.vy  = 0.0								# lateral velocity (m/s)
		self.wz  = 0.0								# yaw rate (rad/s)
		
		self.acc_time_constant = 0.4 # s
		self.df_time_constant  = 0.1 # s

		self.pub_loop()

	def pub_loop(self):
		while not rospy.is_shutdown():
			self._update_vehicle_model()
			
			curr_state = state_est()
			curr_state.header.stamp = rospy.Time.now()
			
			curr_state.x        = self.X
			curr_state.y        = self.Y
			curr_state.psi      = self.psi
			curr_state.v        = (self.vx**2 + self.vy**2)**0.5

			curr_state.v_long   = self.vx
			curr_state.v_lat    = self.vy
			curr_state.yaw_rate = self.wz

			curr_state.a_long   = self.acc
			curr_state.a_lat    = self.a_y
			curr_state.df       = self.df

			self.state_pub.publish(curr_state)
			self.r.sleep()

	def _acc_cmd_callback(self, msg):
		self.tcmd_a = rospy.Time.now()
		self.acc_des = msg.data

	def _df_cmd_callback(self, msg):
		self.tcmd_d = rospy.Time.now()
		self.df_des    = msg.data

	def _update_vehicle_model(self, disc_steps = 10):
		# Genesis Parameters from HCE:
		lf = 1.5213  			# m  	(CoG to front axle)
		lr = 1.4987  			# m  	(CoG to rear axle)
		d  = 0.945	 			# m  	(half-width, currently unused)
		m  = 2303.1   			# kg 	(vehicle mass)
		Iz  = 5520.1			# kg*m2 (vehicle inertia)
		C_alpha_f = 7.6419e4*2  # N/rad	(front axle cornering stiffness) # 200k
		C_alpha_r = 13.4851e4*2	# N/rad	(rear axle cornering stiffness)	 # 250k

		deltaT = self.dt_model/disc_steps
		self._update_low_level_control(self.dt_model)
		for i in range(disc_steps):			

			# Compute tire slip angle
			alpha_f = 0.0
			alpha_r = 0.0
			if math.fabs(self.vx) > 1.0:
				alpha_f = self.df - np.arctan2( self.vy+lf*self.wz, self.vx )
				alpha_r = - np.arctan2( self.vy-lr*self.wz , self.vx)        		
			
			# Compute lateral force at front and rear tire (linear model)
			Fyf = C_alpha_f * alpha_f
			Fyr = C_alpha_r * alpha_r

			# Propagate the vehicle dynamics deltaT seconds ahead.
			# Max with 0 is to prevent moving backwards.
			vx_n  = max(0.0, self.vx  + deltaT * ( self.acc - 1/m*Fyf*np.sin(self.df) + self.wz*self.vy ) )
			
			# Ensure only forward driving.
			if vx_n > 1e-6:
				vy_n  = self.vy  + deltaT * ( 1.0/m*(Fyf*np.cos(self.df) + Fyr) - self.wz*self.vx )
				wz_n  = self.wz  + deltaT * ( 1.0/Iz*(lf*Fyf*np.cos(self.df) - lr*Fyr) )
				a_y = 1.0/m*(Fyf*np.cos(self.df) + Fyr)
			else:
				vy_n = 0.0
				wz_n = 0.0
				a_y = 0.

			psi_n = self.psi + deltaT * ( self.wz )
			X_n   = self.X   + deltaT * ( self.vx*np.cos(self.psi) - self.vy*np.sin(self.psi) )
			Y_n   = self.Y   + deltaT * ( self.vx*np.sin(self.psi) + self.vy*np.cos(self.psi) )


			self.X 	 = X_n
			self.Y   = Y_n
			self.psi = (psi_n + np.pi) % (2.0 * np.pi) - np.pi # https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap
			self.vx  = vx_n
			self.vy  = vy_n
			self.wz  = wz_n
			self.a_y = a_y

	def _update_low_level_control(self, dt_control):
		# e_<n> = self.<n> - self.<n>_des
		# d/dt e_<n> = - kp * e_<n>
		# or kp = alpha, low pass filter gain
		# kp = alpha = discretization time/(time constant + discretization time)
		# This is just to simulate some first order control delay in acceleration/steering.
		self.acc = dt_control/(dt_control + self.acc_time_constant) * (self.acc_des - self.acc) + self.acc
		self.df  = dt_control/(dt_control + self.df_time_constant)  * (self.df_des  - self.df) + self.df

if __name__=='__main__':
	print 'Starting Simulator.'
	try:
		v = VehicleSimulator()
	except rospy.ROSInterruptException:
		pass
