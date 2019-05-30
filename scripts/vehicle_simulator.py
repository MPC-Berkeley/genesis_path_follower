#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32 as float_msg
from genesis_path_follower.msg import state_est
import lk_utils.tiremodel_lib as tm
import pdb

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
		self.tcmd_d_prev = None
		self.acc = 0.0
		self.acc_lon=0.0		# actual acceleration (m/s^2)
		self.acc_lat=0.0
		self.df = 0.0		# actual steering angle (rad)
		self.acc_des = 0.0	# desired acceleration	(m/s^2)
		self.df_des = 0.0	
		self.df_des_prev=0# desired steering_angle (rad)
		self.df_prev=self.df #reference for enforcing steering rate constraint
		self.df_slow=self.df
		self.dt_model = 0.01				# vehicle model update period (s) and frequency (Hz)
		self.hz = int(1.0/self.dt_model)
		self.r = rospy.Rate(self.hz)

		self.steering_delay_modifications=True # Toggle for reverting to default delay model
		self.enable_steering_rate_constr=True

		self.cmd_slow=0  #becomes 1 whenever a steering command is received inside callback fn
		self.max_steering_rate=0.5

		#Delay Dynamics Coeff
		# self.alpha_d=-0.259*1.1
		# self.beta_d=0.95-0.259+0.259*1.1    
		# self.gamma_d=0.2315
		# self.eta_d=0.07495*1.25

		# self.alpha_d=-0.23369298
		# self.beta_d=0.90756935    
		# self.gamma_d=0.2581161
		# self.eta_d=0.07014499

		self.alpha_d=-0.22
		self.beta_d=0.9   
		self.gamma_d=0.3
		self.eta_d=0.0751
		# Simulated Vehicle State.
		self.X   = rospy.get_param('X0', -300.0) 	# X position (m)
		self.Y   = rospy.get_param('Y0', -450.0) 	# Y position (m)
		self.psi = rospy.get_param('Psi0', 0.0) 	# yaw angle (rad)
		self.vx  = rospy.get_param('V0', 1.0)		# longitudinal velocity (m/s)
		self.vy  = 0.0								# lateral velocity (m/s)
		self.wz  = 0.0								# yaw rate (rad/s)
		

		self.acc_time_constant = 0.4 # s
		self.df_time_constant  = 0.1 # s
		# self.df_time_constant  = 0

		self.pub_loop()

	def pub_loop(self):
		while not rospy.is_shutdown():
			self._update_vehicle_model()
			
			curr_state = state_est()
			curr_state.header.stamp = rospy.Time.now()
			curr_state.x   = self.X
			curr_state.y   = self.Y
			curr_state.psi = self.psi
			curr_state.vx  = self.vx
			curr_state.vy  = self.vy
			curr_state.wz  = self.wz
			curr_state.a   = self.acc
			curr_state.df  = self.df
			curr_state.a_lat=self.acc_lat
			curr_state.a_lon=self.acc_lon

			self.state_pub.publish(curr_state)
			self.r.sleep()

	def _acc_cmd_callback(self, msg):
		self.tcmd_a = rospy.Time.now()
		self.acc_des = msg.data

	def _df_cmd_callback(self, msg):
		self.tcmd_d_prev=self.tcmd_d
		self.tcmd_d = rospy.Time.now()
		self.df_des_prev=self.df_des
		self.df_des    = msg.data
		self.df_prev=self.df_slow
		self.df_slow=self.df
		self.cmd_slow=1
		


	def _update_vehicle_model(self, disc_steps = 10):
		# Genesis Parameters from HCE:
		lf = 1.5213  			# m  	(CoG to front axle)
		lr = 1.4987  			# m  	(CoG to rear axle)
		d  = 0.945	 			# m  	(half-width, currently unused)
		m  = 2303.1   			# kg 	(vehicle mass)
		Iz  = 1.0*5520.			# kg*m2 (vehicle inertia)
		C_alpha_f = 200000    # N/rad	(front tire cornering stiffness)
		C_alpha_r = 250000	# N/rad	(rear tire cornering stiffness)
		muF = 0.97     #front friction coeff
		muR = 1.02
		g = 9.81
		Fzf = m*lr*g/(lr+lf)   #Maximum force on front vehicles
		Fzr = m*lf*g/(lr+lf)   #Maximium force on rear vehicles

		deltaT = self.dt_model/disc_steps
		self._update_low_level_control(self.dt_model)
		df_used=self.df#/(1.0+0.1715*self.steering_delay_modifications)
		print self.df
		# pdb.set_trace()
		#self. df, self.df_delay = self.df_delay, self.df

		#print (self.df, self.df_delay)
		for i in range(disc_steps):			

			# Compute tire slip angle
			alpha_f = 0.0
			alpha_r = 0.0
			if math.fabs(self.vx) > 1.0:
				alpha_f = self.df - np.arctan2( self.vy+lf*self.wz, self.vx )
				alpha_r = - np.arctan2( self.vy-lr*self.wz , self.vx)        		
			
			# Compute lateral force at front and rear tire (linear model)
			#Fyf = C_alpha_f * alpha_f
			#Fyr = C_alpha_r * alpha_r
			Fyf = tm.fiala(C_alpha_f, muF, muF, np.array([-alpha_f]), Fzf)
			Fyr = tm.fiala(C_alpha_r, muR, muR, np.array([-alpha_r]), Fzr)

			# Propagate the vehicle dynamics deltaT seconds ahead.
			# Max with 0 is to prevent moving backwards.
			vx_n  = max(0.0, self.vx  + deltaT * ( self.acc - 1/m*Fyf*np.sin(df_used) + self.wz*self.vy ) )
			
			# Ensure only forward driving.
			if vx_n > 1e-6:
				vy_n  = self.vy  + deltaT * ( 1.0/m*(Fyf*np.cos(df_used) + Fyr) - self.wz*self.vx )
				wz_n  = self.wz  + deltaT * ( 1.0/Iz*(lf*Fyf*np.cos(df_used) - lr*Fyr) )
			else:
				vy_n = 0.0
				wz_n = 0.0

			psi_n = self.psi + deltaT * ( self.wz )
			X_n   = self.X   + deltaT * ( self.vx*np.cos(self.psi) - self.vy*np.sin(self.psi) )
			Y_n   = self.Y   + deltaT * ( self.vx*np.sin(self.psi) + self.vy*np.cos(self.psi) )

			noise_vx = np.maximum(-0.05, np.minimum(np.random.randn() * 0.01, 0.05))
			noise_vy = np.maximum(-0.1, np.minimum(np.random.randn() * 0.01, 0.1))
			noise_wz = np.maximum(-0.05, np.minimum(np.random.randn() * 0.005, 0.05))

			self.X 	 = X_n
			self.Y   = Y_n
			self.psi = (psi_n + np.pi) % (2.0 * np.pi) - np.pi # https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap
			self.vx  = vx_n + 0.1*noise_vx
			self.vy  = vy_n + 0.1*noise_vy
			self.wz  = wz_n + 0.1*noise_wz

	def _update_low_level_control(self, dt_control):
		# e_<n> = self.<n> - self.<n>_des
		# d/dt e_<n> = - kp * e_<n>
		# or kp = alpha, low pass filter gain
		# kp = alpha = discretization time/(time constant + discretization time)
		# This is just to simulate some first order control delay in acceleration/steering.
		self.acc = dt_control/(dt_control + self.acc_time_constant) * (self.acc_des - self.acc) + self.acc
				

		#self.df_delay=self.df
		df_des=self.df_des
		if self.steering_delay_modifications is False:
			self.df = dt_control/(dt_control + self.df_time_constant)  * (df_des  - self.df) + self.df
		else:
			# Enforces steering rate limits to adjust commanded steering
			if self.tcmd_d_prev is not None and self.enable_steering_rate_constr is True: 
				time_elapsed=rospy.Time.now()-self.tcmd_d
				# time_elapsed=self.tcmd_d-self.tcmd_d_prev
				max_steering_change=self.max_steering_rate*time_elapsed.to_sec()
				if np.abs(self.df_des-self.df) > max_steering_change:
					df_des=self.df+max_steering_change*(self.df_des-self.df)/np.abs(self.df_des-self.df)
			# Uses second order model for delay
			if self.cmd_slow==0:
				self.df = self.df #ZOH 
			else:
				self.df=self.alpha_d*self.df_prev+self.beta_d*self.df_slow+self.gamma_d*self.df_des_prev+self.eta_d*self.df_des
				self.cmd_slow=0
		

if __name__=='__main__':
	print 'Starting Simulator.'
	try:
		v = VehicleSimulator()
	except rospy.ROSInterruptException:
		pass
