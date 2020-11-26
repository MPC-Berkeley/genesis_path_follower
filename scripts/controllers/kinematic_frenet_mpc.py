# Frenet Nonlinear Kinematic MPC Module.
# Currently only supports a fixed velocity setpoint (initial value of v_ref).

import time
import casadi
import numpy as np
from controller import Controller

class KinFrenetMPCPathFollower(Controller):
	##
	def __init__(self, 
		         N          = 10,            # timesteps in MPC Horizon
		         DT       	= 0.2,           # discretization time between timesteps (s)
		         L_F        = 1.5213,        # distance from CoG to front axle (m)
		         L_R        = 1.4987,        # distance from CoG to rear axle (m)
				 AX_MAX     =  5.0,		
		         AX_MIN     = -10.0,         # min/max longitudinal acceleration constraint (m/s^2) 
				 AY_MAX     =  3.0,		
				 AY_MIN     = -3.0,          # min/max lateral acceleration constraint (m/s^2) 	
		         DF_MAX     =  30*np.pi/180,
		         DF_MIN     = -30*np.pi/180, # min/max front steer angle constraint (rad)
		         AX_DOT_MAX  =  1.5,
		         AX_DOT_MIN  = -1.5,          # min/max longitudinal jerk constraint (m/s^3)
				 AY_DOT_MAX  =  5.,
		         AY_DOT_MIN  = -5.,          # min/max lateral jerk constraint (m/s^3)
		         DF_DOT_MAX =  30*np.pi/180,
		         DF_DOT_MIN = -30*np.pi/180, # min/max front steer angle rate constraint (rad/s)
		         EY_MAX		=  0.8,
				 EY_MIN		= -0.8,          # min/max lateral error bounds (m)
				 EPSI_MAX	=  10*np.pi/180,
				 EPSI_MIN	= -10*np.pi/180, # min/max heading error bounds (rad)
				 Q = [0., 100., 500., 1.],   # weights on s, ey, epsi, v
				 R = [.01, .001]):           # input rate weights on ax, df

		for key in list(locals()):
			if key == 'self':
				pass
			elif key == 'Q':
				self.Q = casadi.diag(Q)
			elif key == 'R':
				self.R = casadi.diag(R)
			else:
				setattr(self, '%s' % key, locals()[key])

		self.opti = casadi.Opti()

		''' 
		(1) Parameters
		'''			
		self.u_prev  = self.opti.parameter(2) # previous input: [u_{acc, -1}, u_{df, -1}]
		self.z_curr  = self.opti.parameter(4) # current state:  [s0, ey_0, epsi_0, v_0]

		# Velocity and Curvature Profile.
		self.curv_ref = self.opti.parameter(self.N)
		self.v_ref    = self.opti.parameter()
		self.z_ref    = casadi.horzcat(casadi.MX.zeros(1,3), self.v_ref)

		'''
		(2) Decision Variables
		'''
		## First index is the timestep k, i.e. self.z_dv[0,:] is z_0.		
		## It has self.N+1 timesteps since we go from z_0, ..., z_self.N.
		## Second index is the state element, as detailed below.
		self.z_dv = self.opti.variable(self.N+1, 4) # s, ey, epsi, v
	
		self.s_dv    = self.z_dv[:, 0]  
		self.ey_dv   = self.z_dv[:, 1]  
		self.epsi_dv = self.z_dv[:, 2]  
		self.v_dv    = self.z_dv[:, 3]  
		
		## Control inputs used to achieve self.z_dv according to dynamics.
		## First index is the timestep k, i.e. self.u_dv[0,:] is u_0.
		## Second index is the input element as detailed below.
		self.u_dv = self.opti.variable(self.N, 2)

		self.acc_dv = self.u_dv[:,0]
		self.df_dv  = self.u_dv[:,1]

		# Slack variables used for lateral acceleration constraints.
		self.sl_ay_dv  = self.opti.variable(self.N)
		
		'''
		(3) Problem Setup: Constraints, Cost, Initial Solve
		'''
		self._add_constraints()

		self._add_cost()	

		self._update_initial_condition(0., 0., 0., 1.)
		
		self._update_reference(self.N*[0.], 5.)
		
		self._update_previous_input(0., 0.)

		self.global_state = np.array([0., 0., 0., 0.])
		self.global_ref   = np.zeros((N+1, 4))
		
		# Ipopt with custom options: https://web.casadi.org/docs/ -> see sec 9.1 on Opti stack.
		p_opts = {'expand': True}
		s_opts = {'max_cpu_time': 0.1, 'print_level': 0} 
		self.opti.solver('ipopt', p_opts, s_opts)

		sol = self.solve()

	def _add_constraints(self):
		## State Bound Constraints
		self.opti.subject_to( self.opti.bounded(self.EY_MIN, self.ey_dv, self.EY_MAX) )
		self.opti.subject_to( self.opti.bounded(self.EPSI_MIN, self.epsi_dv, self.EPSI_MAX) )

		## Initial State Constraint
		self.opti.subject_to( self.s_dv[0]    == self.z_curr[0] )   
		self.opti.subject_to( self.ey_dv[0]   == self.z_curr[1] )  
		self.opti.subject_to( self.epsi_dv[0] == self.z_curr[2] )  
		self.opti.subject_to( self.v_dv[0]    == self.z_curr[3] )   
		
		## State Dynamics Constraints
		for i in range(self.N):
			beta = casadi.atan( self.L_R / (self.L_F + self.L_R) * casadi.tan(self.df_dv[i]) )
			dyawdt = self.v_dv[i] / self.L_R * casadi.sin(beta)
			dsdt = self.v_dv[i] * casadi.cos(self.epsi_dv[i]+beta) / (1 - self.ey_dv[i] * self.curv_ref[i] )   
			ay   = self.v_dv[i] * dyawdt
			
			self.opti.subject_to( self.s_dv[i+1]    == self.s_dv[i]    + self.DT * (dsdt) )  
			self.opti.subject_to( self.ey_dv[i+1]   == self.ey_dv[i]   + self.DT * (self.v_dv[i] * casadi.sin(self.epsi_dv[i] + beta)) ) 
			self.opti.subject_to( self.epsi_dv[i+1] == self.epsi_dv[i] + self.DT * (dyawdt - dsdt * self.curv_ref[i]) )
			self.opti.subject_to( self.v_dv[i+1]    == self.v_dv[i]    + self.DT * (self.acc_dv[i]) )

			self.opti.subject_to( self.opti.bounded(self.AY_MIN - self.sl_ay_dv[i], 
				                                    ay,
				                                    self.AY_MAX + self.sl_ay_dv[i]) )		
            
		## Input Bound Constraints
		self.opti.subject_to( self.opti.bounded(self.AX_MIN, self.acc_dv, self.AX_MAX) )
		self.opti.subject_to( self.opti.bounded(self.DF_MIN, self.df_dv,  self.DF_MAX) )

		# Input Rate Bound Constraints
		self.opti.subject_to( self.opti.bounded( self.AX_DOT_MIN*self.DT, 
			                                     self.acc_dv[0] - self.u_prev[0],
			                                     self.AX_DOT_MAX*self.DT) )

		self.opti.subject_to( self.opti.bounded( self.DF_DOT_MIN*self.DT, 
			                                     self.df_dv[0] - self.u_prev[1],
			                                     self.DF_DOT_MAX*self.DT) )

		for i in range(self.N - 1):
			self.opti.subject_to( self.opti.bounded( self.AX_DOT_MIN*self.DT, 
				                                     self.acc_dv[i+1] - self.acc_dv[i],
				                                     self.AX_DOT_MAX*self.DT) )
			self.opti.subject_to( self.opti.bounded( self.DF_DOT_MIN*self.DT, 
				                                     self.df_dv[i+1]  - self.df_dv[i],
				                                     self.DF_DOT_MAX*self.DT) )
		# Other Constraints
		self.opti.subject_to( 0 <= self.sl_ay_dv ) # nonnegative lateral acceleration slack variable
	
	## Cost function
	def _add_cost(self):
		def _quad_form(z, Q):
			return casadi.mtimes(z, casadi.mtimes(Q, z.T))	
		
		cost = 0
		for i in range(self.N):
			cost += _quad_form(self.z_dv[i, :]-self.z_ref, self.Q)

		for i in range(self.N - 1):
			cost += _quad_form(self.u_dv[i+1, :] - self.u_dv[i,:], self.R)
		
		cost += casadi.sum1(self.sl_ay_dv)
		self.opti.minimize( cost )

	def _get_global_trajectory(self, u_mpc):
		z_mpc = np.ones((self.N+1, 4)) * np.nan
		z_mpc[0, :] = self.global_state

		for ind, u in enumerate(u_mpc):
			x, y, p, v = z_mpc[ind, :]
			u_acc, u_df = u

			beta = np.arctan( self.L_R / (self.L_F + self.L_R) * np.tan(u_df) )

			xn = x + self.DT * (v * np.cos(p + beta))
			yn = y + self.DT * (v * np.sin(p + beta))
			pn = p + self.DT * (v / self.L_R * np.sin(beta))
			vn = v + self.DT * (u_acc)

			z_mpc[ind+1, :] = [xn, yn, pn, vn]

		return z_mpc

	def solve(self):
		st = time.time()
		try:
			sol = self.opti.solve()
			# Optimal solution.
			u_mpc    = sol.value(self.u_dv)
			z_mpc    = sol.value(self.z_dv)
			sl_mpc   = sol.value(self.sl_ay_dv)
			curv_ref = sol.value(self.curv_ref)
			v_ref    = sol.value(self.v_ref)
			is_opt = True
		except:
			# Suboptimal solution (e.g. timed out).
			u_mpc    = self.opti.debug.value(self.u_dv)
			z_mpc    = self.opti.debug.value(self.z_dv)
			sl_mpc   = self.opti.debug.value(self.sl_ay_dv)
			curv_ref = self.opti.debug.value(self.curv_ref)
			v_ref    = self.opti.debug.value(self.v_ref)
			is_opt   = False

		solve_time = time.time() - st
		
		sol_dict = {}
		sol_dict['u_control']    = u_mpc[0,:]      # control input to apply based on solution
		sol_dict['optimal']      = is_opt          # whether the solution is optimal or not
		sol_dict['solve_time']   = solve_time      # how long the solver took in seconds
		sol_dict['u_mpc']        = u_mpc           # solution inputs (N by 2, see self.u_dv above) 
		sol_dict['z_mpc_frenet'] = z_mpc           # solution states (N+1 by 4, see self.z_dv above)
		sol_dict['sl_mpc']       = sl_mpc          # solution slack vars (N by 1, see self.sl_ay_dv above)
		
		sol_dict['v_ref_frenet']    = v_ref        # velocity reference (scalar)
		sol_dict['curv_ref_frenet'] = curv_ref     # curvature reference (N by 1)

		sol_dict['z_mpc'] = self._get_global_trajectory(u_mpc) # get MPC solution trajectory in global frame (N+1, 4)
		sol_dict['z_ref'] = self.global_ref                    # state reference (N by 4)
		

		return sol_dict

	def update(self, update_dict):		
		self.global_state = np.array( [update_dict[key] for key in ['x0', 'y0', 'psi0', 'v0']] )
		self.global_ref   = np.column_stack(( update_dict['x_ref'], 
			                                  update_dict['y_ref'],
			                                  update_dict['psi_ref'],
			                                  update_dict['v_ref'] ))

		self._update_initial_condition( *[update_dict[key] for key in ['s0', 'e_y0', 'e_psi0', 'v0']] )	
		
		# Calculate and apply a velocity reference limit from the upcoming curvatures.
		# This is designed assuming the velocity setpoint is constant and not time-varying.
		v_desired     = update_dict['v_ref'][0]
		curvature_max = np.amax(np.fabs(update_dict['curv_ref']))
		v_limit       = np.sqrt(self.AY_MAX/np.fabs(curvature_max))

		self._update_reference( update_dict['curv_ref'], 
			                    np.clip(v_desired, -v_limit, v_limit ))

		self._update_previous_input( *[update_dict[key] for key in ['acc_prev', 'df_prev']] )

		# TODO: clean up warm start by unifying the keys in sol_dict and update_dict.
		# if 'warm_start' in update_dict.keys():
		# 	# Warm Start used if provided.  Else I believe the problem is solved from scratch with initial values of 0.
		# 	self.opti.set_initial(self.z_dv,  update_dict['warm_start']['z_ws'])
		# 	self.opti.set_initial(self.u_dv,  update_dict['warm_start']['u_ws'])
		# 	self.opti.set_initial(self.sl_dv, update_dict['warm_start']['sl_ws'])

	def _update_initial_condition(self, s0, ey0, epsi0, vel0):
		self.opti.set_value(self.z_curr, [s0, ey0, epsi0, vel0])

	def _update_reference(self, curv_ref, v_ref):
		self.opti.set_value(self.curv_ref, curv_ref)
		self.opti.set_value(self.v_ref, v_ref)

	def _update_previous_input(self, acc_prev, df_prev):
		self.opti.set_value(self.u_prev, [acc_prev, df_prev])

if __name__ == '__main__':
	kmpc = KinFrenetMPCPathFollower()
	sol_dict = kmpc.solve()
	
	np.set_printoptions(precision=3)
	for key in sol_dict:
		print(key, sol_dict[key])
