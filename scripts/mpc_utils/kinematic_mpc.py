import time
import casadi

class KinMPCPathFollower(object):

	def __init__(self, 
		         N          = 10,     # timesteps in MPC Horizon
		         DT         = 0.2,    # discretization time between timesteps (s)
		         L_F        = 1.5213, # distance from CoG to front axle (m)
		         L_R        = 1.4987, # distance from CoG to rear axle (m)
		         V_MIN      = 0.0,    # min/max velocity constraint (m/s)
		         V_MAX      = 20.0,     
		         A_MIN      = -3.0,   # min/max acceleration constraint (m/s^2)
		         A_MAX      =  2.0,     
		         DF_MIN     = -0.5,   # min/max front steer angle constraint (rad)
		         DF_MAX     =  0.5,    
		         A_DOT_MIN  = -1.5,   # min/max jerk constraint (m/s^3)
		         A_DOT_MAX  =  1.5,
		         DF_DOT_MIN = -0.5,   # min/max front steer angle rate constraint (rad/s)
		         DF_DOT_MAX =  0.5, 
		         Q = [1., 1., 10., 0.1],
		         R = [10., 100.]):

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
		self.z_curr  = self.opti.parameter(4) # current state:  [x_0, y_0, psi_0, v_0]

		# Reference trajectory we would like to follow.
		# First index corresponds to our desired state at timestep k+1:
		#   i.e. z_ref[0,:] = z_{desired, 1}.
		# Second index selects the state element from [x_k, y_k, psi_k, v_k].
		self.z_ref = self.opti.parameter(self.N, 4)

		self.x_ref   = self.z_ref[:,0]
		self.y_ref   = self.z_ref[:,1]
		self.psi_ref = self.z_ref[:,2]
		self.v_ref   = self.z_ref[:,3]

		'''
		(2) Decision Variables
		'''
		# Actual trajectory we will follow given the optimal solution.
		# First index is the timestep k, i.e. self.z_dv[0,:] is z_0.		
		# It has self.N+1 timesteps since we go from z_0, ..., z_self.N.
		# Second index is the state element, as detailed below.
		self.z_dv = self.opti.variable(self.N+1, 4)
	
		self.x_dv   = self.z_dv[:, 0]
		self.y_dv   = self.z_dv[:, 1]
		self.psi_dv = self.z_dv[:, 2]
		self.v_dv   = self.z_dv[:, 3]

		# Control inputs used to achieve self.z_dv according to dynamics.
		# First index is the timestep k, i.e. self.u_dv[0,:] is u_0.
		# Second index is the input element as detailed below.
		self.u_dv = self.opti.variable(self.N, 2)

		self.acc_dv = self.u_dv[:,0]
		self.df_dv  = self.u_dv[:,1]

		# Slack variables used to relax input rate constraints.
		# Matches self.u_dv in structure but timesteps range from -1, ..., N-1.
		self.sl_dv  = self.opti.variable(self.N , 2)
		
		self.sl_acc_dv = self.sl_dv[:,0]
		self.sl_df_dv  = self.sl_dv[:,1]
		
		'''
		(3) Problem Setup: Constraints, Cost, Initial Solve
		'''
		self._add_constraints()
		
		self._add_cost()
		
		self.update_initial_condition(0., 0., 0., 1.)
		
		self.update_reference([self.DT * (x+1) for x in range(self.N)],
			                  self.N*[0.], 
			                  self.N*[0.], 
			                  self.N*[1.])
		
		self.update_previous_input(0., 0.)
		
		# Ipopt with custom options: https://web.casadi.org/docs/ -> see sec 9.1 on Opti stack.
		p_opts = {'expand': True}
		s_opts = {'max_cpu_time': 0.1, 'print_level': 0} 
		self.opti.solver('ipopt', p_opts, s_opts)

		sol = self.solve()


	def _add_constraints(self):
		# State Bound Constraints
		self.opti.subject_to( self.opti.bounded(self.V_MIN, self.v_dv, self.V_MAX) )		

		# Initial State Constraint
		self.opti.subject_to( self.x_dv[0]   == self.z_curr[0] )
		self.opti.subject_to( self.y_dv[0]   == self.z_curr[1] )
		self.opti.subject_to( self.psi_dv[0] == self.z_curr[2] )
		self.opti.subject_to( self.v_dv[0]   == self.z_curr[3] )

		# State Dynamics Constraints
		for i in range(self.N):
			beta = casadi.atan( self.L_R / (self.L_F + self.L_R) * casadi.tan(self.df_dv[i]) )
			self.opti.subject_to( self.x_dv[i+1]   == self.x_dv[i]   + self.DT * (self.v_dv[i] * casadi.cos(self.psi_dv[i] + beta)) )
			self.opti.subject_to( self.y_dv[i+1]   == self.y_dv[i]   + self.DT * (self.v_dv[i] * casadi.sin(self.psi_dv[i] + beta)) )
			self.opti.subject_to( self.psi_dv[i+1] == self.psi_dv[i] + self.DT * (self.v_dv[i] / self.L_R * casadi.sin(beta)) )
			self.opti.subject_to( self.v_dv[i+1]   == self.v_dv[i]   + self.DT * (self.acc_dv[i]) )

		# Input Bound Constraints
		self.opti.subject_to( self.opti.bounded(self.A_MIN,  self.acc_dv, self.A_MAX) )
		self.opti.subject_to( self.opti.bounded(self.DF_MIN, self.df_dv,  self.DF_MAX) )

		# Input Rate Bound Constraints
		self.opti.subject_to( self.opti.bounded( self.A_DOT_MIN   -  self.sl_acc_dv[0], 
			                                     self.acc_dv[0] - self.u_prev[0],
			                                     self.A_DOT_MAX   + self.sl_acc_dv[0]) )

		self.opti.subject_to( self.opti.bounded( self.DF_DOT_MIN  -  self.sl_df_dv[0], 
			                                     self.df_dv[0] - self.u_prev[1],
			                                     self.DF_DOT_MAX  + self.sl_df_dv[0]) )

		for i in range(self.N - 1):
			self.opti.subject_to( self.opti.bounded( self.A_DOT_MIN   -  self.sl_acc_dv[i+1], 
				                                     self.acc_dv[i+1] - self.acc_dv[i],
				                                     self.A_DOT_MAX   + self.sl_acc_dv[i+1]) )
			self.opti.subject_to( self.opti.bounded( self.DF_DOT_MIN  -  self.sl_df_dv[i+1], 
				                                     self.df_dv[i+1]  - self.df_dv[i],
				                                     self.DF_DOT_MAX  + self.sl_df_dv[i+1]) )
		# Other Constraints
		self.opti.subject_to( 0 <= self.sl_df_dv )
		self.opti.subject_to( 0 <= self.sl_acc_dv )
		# e.g. things like collision avoidance or lateral acceleration bounds could go here.

	def _add_cost(self):
		def _quad_form(z, Q):
			return casadi.mtimes(z, casadi.mtimes(Q, z.T))

		cost = 0
		for i in range(self.N):
			cost += _quad_form(self.z_dv[i+1, :] - self.z_ref[i,:], self.Q)

		for i in range(self.N - 1):
			cost += _quad_form(self.u_dv[i+1, :] - self.u_dv[i,:], self.R)

		cost += (casadi.sum1(self.sl_df_dv) + casadi.sum1(self.sl_acc_dv))

		self.opti.minimize( cost )

	def solve(self, z_dv_warm_start = None, u_dv_warm_start = None, sl_dv_warm_start = None):
                # Warm Start used if provided.  Else I believe the problem is solved from scratch with initial values of 0.
		if z_dv_warm_start is not None:
			self.opti.set_initial(self.z_dv, z_dv_warm_start)
		if u_dv_warm_start is not None:
			self.opti.set_initial(self.u_dv, u_dv_warm_start)
		if sl_dv_warm_start is not None:
			self.opti.set_initial(self.sl_dv, sl_dv_warm_start)

		st = time.time()
		try:
			sol = self.opti.solve()
			# Optimal solution.
			u_opt  = sol.value(self.u_dv)
			z_opt  = sol.value(self.z_dv)
			sl_opt = sol.value(self.sl_dv)
			z_ref  = sol.value(self.z_ref)
			is_opt = True
		except:
			# Suboptimal solution.
			u_opt  = self.opti.debug.value(self.u_dv)
			z_opt  = self.opti.debug.value(self.z_dv)
			sl_opt = self.opti.debug.value(self.sl_dv)
			z_ref  = self.opti.debug.value(self.z_ref)

			is_opt = False

		solve_time = time.time() - st
		
		# stats = sol.stats()
		# is_opt = stats['success']
		# TODO: could also use stats wall clock time.

		return is_opt, solve_time, u_opt, z_opt, sl_opt, z_ref

	def update_initial_condition(self, x0, y0, psi0, vel0):
		self.opti.set_value(self.z_curr, [x0, y0, psi0, vel0])

	def update_reference(self, x_ref, y_ref, psi_ref, v_ref):
		self.opti.set_value(self.x_ref,   x_ref)
		self.opti.set_value(self.y_ref,   y_ref)
		self.opti.set_value(self.psi_ref, psi_ref)
		self.opti.set_value(self.v_ref,   v_ref)

	def update_previous_input(self, acc_prev, df_prev):
		self.opti.set_value(self.u_prev, [acc_prev, df_prev])

if __name__ == '__main__':
	kmpc = KinMPCPathFollower()
	sol = kmpc.solve()
	import pdb; pdb.set_trace()
