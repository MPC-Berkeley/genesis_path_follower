#!/usr/bin/env julia

#=
 Note: This version is provided for reference, but has not been tested with the Genesis.

 This is a modified version of controller_MPC_traj.jl from the barc project ROS codebase.
 Modified by Vijay Govindarajan.

 Licensing Information: You are free to use or extend these projects for 
 education or research purposes provided that (1) you retain this notice
 and (2) you provide clear attribution to UC Berkeley, including a link 
 to http://barc-project.com

 Attibution Information: The barc project ROS code-base was developed
 at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
 (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
 by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
 based on an open source project by Bruce Wootton
=# 

module GPSKinMPCPathFollowerFrenet
	__precompile__()
    using JuMP
    using Ipopt

    #### (1) Initialize model and model parameters and MPC gains ####
	dt_control = 0.10		# control period, ts (s)
    mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time = dt_control))

	L_a     = 1.108 		# dist from CoG to front axle (m)
	L_b     = 1.742 		# dist from CoG to rear axle (m)
	dt      = 0.20			# model discretization time, td (s)
	N       = 8				# horizon
   
    path_ref = Dict() # placeholder to keep the reference set of waypoints for logging.
	v_ref = 15.0						# target velocity for path follower
	k_coeff_ref = [0.0, 0.0, 0.0, 0.0]	# K(s) = a0 + a1 * s + a2 * s^2 + a3 * s^3
										# [a3,a2,a1,a0] -> highest degree first!

    steer_max = 0.5 		# tire angle (d_f) bound, rad
    steer_dmax = 0.5		# tire angle (d_f) rate bound, rad/s

	a_max = 2.0				# acceleration and deceleration bound, m/s^2
	a_dmax = 1.5			# jerk bound, m/s^3

	v_min = 0.0				# vel bounds (m/s)
	v_max = 20.0			
	

    # Cost function gains.
    C_ey = 10.0				# lateral deviation
    C_epsi = 5.0			# heading deviation
    C_ev   = 0.5			# target velocity deviation

	C_dacc	 = 0.1			# derivative of acceleration input
	C_ddf	 = 3e4		# derivative of tire angle input
	C_acc	 = 4.0			# acceleration input
	C_df	 = 150			# tire angle input

	#### (2) Define State/Input Variables and Constraints ####
	# states: position (x,y), velocity (v), heading (psi)
	# inputs: tire angle (d_f) and acceleration (acc).
	println("Creating kinematic bicycle model ....")
	@variable( mdl, s[1:(N+1)], start=0.0)
	@variable( mdl, ey[1:(N+1)], start=0.0)
	@variable( mdl, epsi[1:(N+1)], start=0.0)
	@variable( mdl, v_min <= v[1:(N+1)] <= v_max, start=0.0)


	# Input Constraints
	@variable( mdl, -a_max <= acc[1:N] <= a_max, start=0.0)
	@variable( mdl, -steer_max <= d_f[1:N] <= steer_max, start=0.0)

	# Input Steering Rate Constraints
	@NLparameter(mdl, d_f_current == 0.0) # Current tire angle is a model parameter.
	@NLconstraint(mdl, -steer_dmax*dt_control <= d_f[1]  - d_f_current <= steer_dmax*dt_control)
    for i in 2:(N-1)
        @constraint(mdl, -steer_dmax*dt <= d_f[i+1] - d_f[i] <= steer_dmax*dt)
    end

	# Input Acceleration Rate Constraints
	@NLparameter(mdl, acc_current == 0.0) # Current acceleration is a model parameter.
	@NLconstraint(mdl, -a_dmax*dt_control <= acc[1]  - acc_current <= a_dmax*dt_control)
    for i in 2:(N-1)
        @constraint(mdl, -a_dmax*dt <= acc[i+1] - acc[i] <= a_dmax*dt)
    end	

	#### (3) Define Objective ####

	# Reference trajectory is encoded as model parameters.
	@NLparameter(mdl, k_poly[i=1:length(k_coeff_ref)] == k_coeff_ref[i]) 
	@NLparameter(mdl, v_target == v_ref)

	# Cost function.
    @NLobjective(mdl, Min, sum{ C_ey*(ey[i])^2 + C_epsi*(epsi[i])^2 , i=2:(N+1)} + 
                                C_ev*sum{ (v[i] - v_target)^2, i = 2:N} + 
                                C_acc*sum{(acc[i])^2, i=1:N} +
                                C_df*sum{(d_f[i])^2, i=1:N} +
				                C_dacc*sum{(acc[i+1] - acc[i])^2, i=1:(N-1)} +
                                C_ddf*sum{(d_f[i+1] - d_f[i])^2, i=1:(N-1)}
				)

	#### (4) Define System Dynamics Constraints ####
	# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
	#               Spring, 2011, page 26

    # Initial condition is a model parameter.
	@NLparameter(mdl, s0    == 0.0);  @NLconstraint(mdl, s[1]   == s0);    
	@NLparameter(mdl, ey0   == 0.0);  @NLconstraint(mdl, ey[1]   == ey0);    
	@NLparameter(mdl, epsi0 == 0.0);  @NLconstraint(mdl, epsi[1] == epsi0);
    @NLparameter(mdl, v0    == 0.0);  @NLconstraint(mdl, v[1]   == v0);

	@NLexpression(mdl, K[i = 1:N],    k_poly[1]*s[i]^3 + k_poly[2]*s[i]^2 + k_poly[3]*s[i] + k_poly[4]) # TODO: CHECK ORDER MAKES SENSE.
	@NLexpression(mdl, bta[i = 1:N],  atan( L_b / (L_a + L_b) * tan( d_f[i]) ) )
	@NLexpression(mdl, dsdt[i = 1:N], v[i]*cos(epsi[i] + bta[i]) / (1-ey[i]*K[i]) )

	for i in 1:N
		# equations of motion wrt CoG
    	@NLconstraint(mdl, s[i+1]     == s[i]       + dt*( dsdt[i] ))
    	@NLconstraint(mdl, ey[i+1]    == ey[i]      + dt*( v[i]*sin(epsi[i]+bta[i]) ))
    	@NLconstraint(mdl, epsi[i+1]  == epsi[i]    + dt*( v[i]/L_b*sin(bta[i]) - dsdt[i]*K[i] ))
    	@NLconstraint(mdl, v[i+1]     == v[i]       + dt*( acc[i] ))
	end

    #### (5) Initialize Solver ####
	println("MPC: Initial solve ...")
	status = solve(mdl)
	println("MPC: Finished initial solve: ", status)

	#####################################	
	##### State Update Function #####
    function update_init_cond(s::Float64, ey::Float64, epsi::Float64, vel::Float64)
        # update mpc initial condition 
        setvalue(s0,    s)
        setvalue(ey0,   ey)
        setvalue(epsi0, epsi)
        setvalue(v0,    vel)
    end

	#####################################
	##### Reference Update Function #####
    function update_reference(path::Dict, k_coeffs::Array{Float64,1}, v_des::Float64)
    	global path_ref
    	path_ref = path
    	setvalue(k_poly[i=1:4], k_coeffs) # Reference trajectory can be updated.
    	setvalue(v_target, v_des)
    end

	#########################################
	##### Input Update Function #####
	function update_current_input(c_swa::Float64, c_acc::Float64)
		setvalue(d_f_current, c_swa)
		setvalue(acc_current, c_acc)
	end

	#################################
	##### Model Solve Function #####
    function solve_model()
        # Solve the model, assuming relevant update functions have been called by the user.

        status = solve(mdl)

        # get optimal solutions
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])
        return acc_opt[1], d_f_opt[1], status
    end

	#################################
	##### Diagnostics Function ######
	function get_solver_results()
		# This function gets all relevant solver variables and returns it to the client.
		# Handy for debugging or logging full results.

		# State Variables and Reference
		s_mpc   = getvalue(s[1:(N+1)])
		ey_mpc   = getvalue(ey[1:(N+1)])
		v_mpc   = getvalue(v[1:(N+1)])
		epsi_mpc = getvalue(epsi[1:(N+1)])
		K = getvalue(k_poly[1:4])

		# Optimal Solution
        d_f_opt = getvalue(d_f[1:N])
        acc_opt = getvalue(acc[1:N])

        # path_ref set in update_reference function.
		return s_mpc, ey_mpc, v_mpc, epsi_mpc, K, path_ref, d_f_opt, acc_opt	
	end
end
