#!/usr/bin/env julia

#=
 This is a modified version of controller_MPC.jl from the barc project ROS codebase.
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
__precompile__()

module GPSDynMPCPathFollower
	import DynBicycleModel
	const dyn_fncn = DynBicycleModel.f_dyn_bicycle_model
	const jac_dyn_fncn = DynBicycleModel.jacobian_f_dyn_bicycle_model

    using JuMP
    using Ipopt

    #### (1) Initialize model and model parameters and MPC gains ####
	dt_control = 0.04		# control period, ts (s)
    mdl = Model(solver = IpoptSolver(print_level=0, max_cpu_time = dt_control))

	dt      = 0.10			# model discretization time, td (s)
	N       = 16			# horizon
   
	v_ref = 1.0							    # target velocity for path follower
	x_ref   = v_ref*collect(0.0:dt:N*dt)	# target x(0.0 s), ..., x(N*dt s)
	y_ref   = zeros(N+1)					# target y(0.0 s), ..., y(N*dt s)
    psi_ref = zeros(N+1)					# target psi(0.0 s), ..., psi(N*dt s)
	
    steer_max = 0.4 		# tire angle (d_f) bound, rad
    steer_dmax = 0.4		# tire angle (d_f) rate bound, rad/s

	a_min = -3.0			# acceleration and deceleration bounds, m/s^2
	a_max = 2.0				
	a_dmax = 1.5			# jerk bound, m/s^3

	v_min = 0.0				# vel bounds (m/s)
	v_max = 20.0			
	
        # Cost function gains.
	@NLparameter(mdl, C_x    == 9.0) # longitudinal deviation
	@NLparameter(mdl, C_y    == 9.0) # lateral deviation
	@NLparameter(mdl, C_psi  == 10.0) # heading deviation
	@NLparameter(mdl, C_v    == 0.0) # target velocity deviation

	@NLparameter(mdl, C_dacc == 100.0) # jerk
	@NLparameter(mdl, C_ddf  == 1000.0) # slew rate
	@NLparameter(mdl, C_acc  == 0.0)   # acceleration
	@NLparameter(mdl, C_df   == 0.0)   # tire angle

	#### (2) Define State/Input Variables and Constraints ####
	# States: z in R^6, i in 1,...,(N+1)
	# z[i,1] = X[i], E position in m
	# z[i,2] = Y[i], N position in m
	# z[i,3] = psi[i], yaw wrt E-axis in radians
	# z[i,4] = vx[i], longitudinal velocity (m/s)
	# z[i,5] = vy[i], lateral velocity (m/s)
	# z[i,6] = wz[i], angular velocity (rad/s)

	# z[i,:] = z_nom[i,:] + dz[i,:]
	#dz[1,:] = zeros -> no deviation from z_nom at the first timestep
	@variable( mdl, dz[1:(N+1), 1:6], start=0.0)
	@variable( mdl, z[1:(N+1), 1:6], start=0.0)
	@NLparameter(mdl, z_nom[1:(N+1), 1:6] == 0.0)
	for i = 1:(N+1)
		for j = 1:6
			@NLconstraint(mdl, z[i, j] == z_nom[i,j] + dz[i,j])
		end
		@NLconstraint(mdl, v_min <= z[i,4] <= v_max)
	end
	for j in 1:6
		@constraint(mdl, 0.0 <= dz[1,j] <= 0.0)
	end

	# Inputs: u in R^2, i in 1,...,N
	# u[i,1] = acc[i], tire angle (d_f) in radians
	# u[i,2] = d_f[i], acceleration in m/s^2
	@NLparameter(mdl, u_nom[i=1:N, j=1:2] == 0.0)
	@variable( mdl, du[1:N,1:2], start=0.0)
	@variable( mdl, u[1:N,1:2], start=0.0)
	for i = 1:N
		for j = 1:2
			@NLconstraint(mdl, u[i, j] == u_nom[i,j] + du[i,j])
		end
	end

	# Input Constraints
	for j in 1:N
		@NLconstraint(mdl, a_min <= u[j,1] <= a_max)
		@NLconstraint(mdl, -steer_max <= u[j,2] <= steer_max)
	end

	# Input Acceleration Rate Constraints
	@NLconstraint(mdl, -a_dmax*dt_control <= du[1,1] <= a_dmax*dt_control)
    for i in 1:(N-1)
        @constraint(mdl, -a_dmax*dt <= u[i+1,1]-u[i,1] <= a_dmax*dt)
    end	

	# Input Steering Rate Constraints
	@NLconstraint(mdl, -steer_dmax*dt_control <= du[1,2] <= steer_dmax*dt_control)
    for i in 1:(N-1)
        @constraint(mdl, -steer_dmax*dt <= u[i+1,2] - u[i,2] <= steer_dmax*dt)
    end

	#### (3) Define Objective ####

	# Reference trajectory is encoded as model parameters.
	@NLparameter(mdl, z_r[1:(N+1), 1:6] == 0.0) 

	# Cost function.
    @NLobjective(mdl, Min, C_x    * sum{ (z[i,1]-z_r[i,1])^2, i=2:(N+1)} + 
    					   C_y    * sum{ (z[i,2]-z_r[i,2])^2, i=2:(N+1)} + 
    					   C_psi  * sum{ (z[i,3]-z_r[i,3])^2, i=2:(N+1)} + 
                           C_v    * sum{ (z[i,4]-z_r[i,4])^2, i=2:(N+1)}    + 
                           C_acc  * sum{ (u[i,1])^2, i=1:N}                +
                           C_df   * sum{ (u[i,2])^2, i=1:N}                +
						   C_dacc * sum{ (u[i+1,1] - u[i,1])^2,  i=1:(N-1)} +
                           C_ddf  * sum{ (u[i+1,2] - u[i,2])^2,  i=1:(N-1)}
				)

	#### (4) Define System Dynamics Constraints ####
	# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
	#               Spring, 2011, page 26

	@NLparameter(mdl, A[i=1:N, 1:6, 1:6] == 0.0) # df/dx[i]
	@NLparameter(mdl, B[i=1:N, 1:6, 1:2] == 0.0) # df/du[i]

	for i in 1:N # timestep
		# this outer for loop should iterate over the timestep in the open-loop plan.
		for j in 1:6 # state
	        # equations of motion wrt CoG
			# this inner for loop should iterate over the state (i.e. operate over rows of A[i,:,:] and B[i,:,:] to fill out z[:,i+1].
			@NLconstraint(mdl, dz[i+1,j]   == dz[i,j]   + dt*( sum{ A[i,j,k]*dz[i,k] , k=1:6 } + sum{ B[i,j,m]*du[i,m] , m=1:2 }) )
		end
	end

    #### (5) Initialize Solver ####
	status = solve(mdl)
	
	#####################################
	##### Reference Update Function #####
    function update_reference(z_ref::Array{Float64,2})
    	for i = 1:(N+1)
    		for j = 1:6
    			setvalue(z_r[i,j], z_ref[i,j])
    		end
    	end
    end

	#############################################
	##### Model Update Function ###

	function update_model(z_curr::Array{Float64,1}, u_curr::Array{Float64,1}) # SINGLE INPUT CASE
		# Compute nominal trajectory
		for j = 1:6
			setvalue(z_nom[1,j], z_curr[j])
		end

		for i = 1:N
			lin_point = vec([z_curr;u_curr])
			f = dyn_fncn( lin_point  )
			J = jac_dyn_fncn( lin_point )
			Al = J[:,1:6]
			Bl = J[:,7:8]

			z_curr = convert(Array{Float64,1}, z_curr + f * dt )

			for j = 1:6
				setvalue(z_nom[i+1,j], z_curr[j])
				for k = 1:6
					setvalue(A[i,j,k],Al[j,k])
				end
				for k = 1:2
					setvalue(B[i,j,k],Bl[j,k])
				end
			end

			for j = 1:2
				setvalue(u_nom[i,j], u_curr[j])
			end
		end

	end

	function update_model(z_curr::Array{Float64,1}, u_curr::Array{Float64,2}) # INPUT TRAJECTORY CASE
		# Compute nominal trajectory
		for j = 1:6
			setvalue(z_nom[1,j], z_curr[j])
		end

		for i = 1:N
			if i < N
				u = vec(u_curr[i+1,:])
			else
				u = vec(u_curr[end,:])
			end
				
			lin_point = vec([ z_curr;u])
			f = dyn_fncn( lin_point  )
			J = jac_dyn_fncn( lin_point )
			Al = J[:,1:6]
			Bl = J[:,7:8]

			z_curr = convert(Array{Float64,1}, z_curr + f * dt )

			for j = 1:6
				setvalue(z_nom[i+1,j], z_curr[j])
				for k = 1:6
					setvalue(A[i,j,k],Al[j,k])
				end
				for k = 1:2
					setvalue(B[i,j,k],Bl[j,k])
				end
			end

			for j = 1:2
				setvalue(u_nom[i,j], u[j])
			end
		end

	end


	#########################################
	##### Cost Update Function #####
	function update_cost(cx::Float64, cy::Float64, cp::Float64, cv::Float64,
						 cda::Float64, cdd::Float64, ca::Float64, cd::Float64)
		setvalue(C_x,   cx)
		setvalue(C_y,   cy) 
		setvalue(C_psi, cp)
		setvalue(C_v,   cv)

		setvalue(C_dacc, cda)
		setvalue(C_ddf,  cdd)
		setvalue(C_acc,  ca)
		setvalue(C_df,   cd)
	end

	#################################
	##### Model Solve Function #####
    function solve_model()
        # Solve the model, assuming relevant update functions have been called by the user.
        tic()
        status = solve(mdl)
        solv_time = toq()

        # get optimal solutions
        acc_opt = getvalue(u[1:N,1])
        d_f_opt = getvalue(u[1:N,2])
        return acc_opt[1], d_f_opt[1], status, solv_time
    end

	#################################
	##### Diagnostics Function ######
	function get_solver_results()
		# This function gets all relevant solver variables and returns it to the client.
		# Handy for debugging or logging full results.
		# State Variables and Reference
		z_mpc   = getvalue(z[1:(N+1), 1:6])
		z_ref   = getvalue(z_r[1:(N+1), 1:6])
		
		# Optimal Solution
        acc_opt = getvalue(u[1:N,1])
        d_f_opt = getvalue(u[1:N,2])

		return z_mpc, z_ref, d_f_opt, acc_opt	
	end
end
