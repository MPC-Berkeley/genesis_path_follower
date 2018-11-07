#!/usr/bin/env julia

#=
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

#=
	Frenet Pathfollowing that uses linear approximation
		- this module is concerned with Longitudinal control
		- tracks (s_ref, v_ref)
		- NOTE: BOX constraint on "s" (using largeNumber) is not well-implemented atm
=#
###### ALL STATE CONSTRAINTS RELAXED


module GPSKinMPCPathFollowerFrenetLinLongGurobi
	__precompile__()

	using Gurobi
	import KinMPCParams 		# load basic parameters such as N, dt, L_a, L_b that is shared among the controllers


	println("Creating longitudinal kinematic bicycle model in Gurobi/OSQP ....")

 
	# ====================== general problem formulation is given by ======================
	# x_{k+1} = A x_k + B u_k + g_k
	# u_lb <= u_k <= u_ub
	# x_lb <= x_k <= x_ub
	# dU_lb <= u_k - u_{k-1} <= dU_ub
	# minimize (x_k - x_k_ref)' * Q * (x_k - x_k_ref) + (u_k - u_k_ref)' * R * (u_k - u_k_ref) + (u_k - u_{k-1})' * Rdelta * (u_k - u_{k-1}) 


	# general control parameters
	# some might be passed on as arguments
	# general control parameters
	dt      = KinMPCParams.dt		# model discretization time, td (s)
	N       = KinMPCParams.N		# horizon, N=8 is standard
	L_a 	= KinMPCParams.L_a		# from CoG to front axle (according to Jongsang)
	L_b 	= KinMPCParams.L_b				# from CoG to rear axle (according to Jongsang)

	nx 		= 2				# dimension of x = (s,v)
	nu 		= 1				# number of inputs u = a

	# define System matrices (all time-invariant)
	A = [	1 	dt 		# can be made more exact using matrix exponential
			0	1 	]
	B = [ 	0
			dt 		]
	g = zeros(nx)

	# define cost functions
    C_s = KinMPCParams.C_s			# track progress
	C_v = KinMPCParams.C_v			# ref velocity tracking weight			
	C_acc = KinMPCParams.C_acc
	C_dacc = KinMPCParams.C_dacc		# 20 too high; 10 OK for med speed; 10 a big jerky for high speed; 13 too high

	Q = diagm([C_s ; C_v])	# create diagonal matrix
	R = C_acc
	Rdelta = C_dacc

	# define (box) constraints
	largeNumber = 1e5;		# use this number for variables that are not upper/lower bounded
	v_min = KinMPCParams.v_min				# vel bounds (m/s)
	v_max = KinMPCParams.v_max	
	a_max = KinMPCParams.a_max				# acceleration and deceleration bound, m/s^2
	a_dmax = KinMPCParams.a_dmax			# jerk bound, m/s^3

	x_lb = [		-largeNumber	# make sure car doesnt travel more than largeNumber [m]
				    v_min	] 		# v_min
	x_ub = [		largeNumber
					v_max		]

	u_lb = -a_max #-a_max
	u_ub =  a_max  # a_max

	dU_lb = -a_dmax*dt 	# double check if *dt is needed (or not)
	dU_ub = a_dmax*dt


	# build references (should be passed on as arguments later on)
	# get init x_ref for one initial solve (to speed up Julia)
    s_ref_init   = 1.0*collect(dt:dt:(N)*dt)		# target s(1), ..., s(N)
	v_ref_init = ones(N,1)						    # reference velocity 
	x_ref_init = zeros(N*nx,1)
	for i = 1 : N
		x_ref_init[(i-1)*nx+1] = s_ref_init[i]		# set x_ref
		x_ref_init[(i-1)*nx+2] = v_ref_init[i]		# set v_ref
	end

	# input reference
	u_ref_init = zeros(N,1)	# if not used, set cost to zeros


	# get Initial state and input
	# should be done dynamically later on
	s0_init = 0
	v0_init = 0
	x0_init = [s0_init ; v0_init]
	u0_init = 0


	# ================== Transformation 1 ======================
	# augment state and redefine system dynamics (A,B,g) and constraints
	# x_tilde_k := (x_k , u_{k-1})
	# u_tilde_k := (u_k - u_{k-1})

	A_tilde = [	A 				B
				zeros(nu,nx) 	eye(nu)	]

	B_tilde = [	B ; eye(nu)	]

	g_tilde = [	g ;	zeros(nu) ]

	x_tilde_lb = [x_lb ; u_lb]
	x_tilde_ub = [x_ub ; u_ub]
	u_tilde_lb = dU_lb
	u_tilde_ub = dU_ub

	Q_tilde = [	Q 			zeros(nx,nu) 		# may also use cat(?)
				zeros(nu,nx)	R 			]	# actually not needed

	R_tilde = Rdelta

	x_tilde_0_init = [x0_init ; u0_init]	# initial state of system; PARAMETER

	x_tilde_ref_init = zeros(N*(nx+nu))
	for i = 1 : N
		x_tilde_ref_init[(i-1)*(nx+nu)+1 : (i-1)*(nx+nu)+nx] = x_ref_init[(i-1)*nx+1 : i*nx]
		x_tilde_ref_init[(i-1)*(nx+nu)+nx+1 : (i-1)*(nx+nu)+nx+nu] = u_ref_init[i]
	end

	u_tilde_ref_init = zeros(N*nu) 	# goal is to minimize uTilde = (acc_k - acc_{k-1})

	# ================== Transformation 2 ======================
	# bring into GUROBI format
	# minimize_z    z' * H * z + f' * z
	#	s.t.		A_eq * z = b_eq
	#				A * z <= b
	#				z_lb <= z <= z_ub

	# z := (u_tilde_0, x_tilde_1 , u_tilde_1 x_tilde_2 , ... u_tilde_{N-1}, x_tilde_N , )
	
	n_uxu = nu+nx+nu 	# size of one block of (u_tilde, x_tilde) = (deltaU, x, u)

	# Build cost function
	# cost for (u_tilde, x_tilde) = (deltaU , S, V, U)
	H_block = [	R_tilde zeros(nu, nu+nx)
				zeros(nu+nx,nu) Q_tilde		];
	H_gurobi = kron(eye(N), H_block)


	z_gurobi_ref_init = zeros(N*n_uxu) 	# reference point for z_gurobi ; PARAMETER!
	for i = 1 : N
		z_gurobi_ref_init[(i-1)*n_uxu+1 : (i-1)*n_uxu+nu] = u_tilde_ref_init[(i-1)*nu+1 : i*nu] 		# should be zero for this application
		z_gurobi_ref_init[(i-1)*n_uxu+nu+1 : i*n_uxu] = x_tilde_ref_init[(i-1)*(nx+nu)+1 : i*(nu+nx)]
	end 


	f_gurobi_init = -2*H_gurobi*z_gurobi_ref_init


	# build box constraints lb_gurobi <= z <= ub_gurobi
	# recall: z = (u_tilde, x_tilde, ....)
	lb_gurobi = repmat([u_tilde_lb ; x_tilde_lb], N, 1)		# (deltaU, X, U)
	ub_gurobi = repmat([u_tilde_ub ; x_tilde_ub], N, 1)		# (deltaU, X, U)


	# build equality matrix (most MALAKA task ever)
	nu_tilde = nu
	nx_tilde = nu+nx
	# n_uxu = nu_tilde + nx_tilde
	Aeq_gurobi = zeros(N*nx_tilde , N*(nx_tilde+nu_tilde))
	Aeq_gurobi[1:nx_tilde, 1:(nx_tilde+nu_tilde)] = [-B_tilde eye(nx_tilde)] 	# fill out first row associated with x_tilde_1
	for i = 2 : N  	# fill out rows associated to x_tilde_2, ... , x_tilde_N
		Aeq_gurobi[ (i-1)*nx_tilde+1 : i*nx_tilde  , (i-2)*(nu_tilde+nx_tilde)+(nu_tilde)+1 : (i-2)*(nu_tilde+nx_tilde)+nu_tilde+(nx_tilde+nu_tilde+nx_tilde)    ] = [-A_tilde -B_tilde eye(nx_tilde)]
	end

	# right-hand-size of equality constraint
	beq_gurobi = repmat(g_tilde,N,1);
	beq_gurobi[1:nx_tilde] = beq_gurobi[1:nx_tilde] + A_tilde*x_tilde_0_init 	# PARAMETER: depends on x0


	# ================ Solve Problem =================== 
    tic()
    GurobiEnv = Gurobi.Env()
	setparam!(GurobiEnv, "Presolve", 0)	# # set presolve to 0 what does it mean?
	setparam!(GurobiEnv, "LogToConsole", 0)	# # set presolve to 0 what does it mean?

	# add A = A_gurobi and b=b_gurobi for inequality constraint
	# note that: (1/2) * z' * H * z + f' * z
    GurobiModel = gurobi_model(GurobiEnv;
    			name = "qp_01",
    			H = 2*H_gurobi,
    			f = f_gurobi_init,	# PARAMETER that depends on x_ref and u_ref
    			Aeq = Aeq_gurobi,
    			beq = squeeze(beq_gurobi,2),	# PARAMETER that depends on x0, u_{-1}	
    			lb = squeeze(lb_gurobi,2),
    			ub = squeeze(ub_gurobi,2)	)
    optimize(GurobiModel)
	solv_time=toq()
	println("1st solv time Gurobi:  $(solv_time*1000) ms")

	# # access results
	# sol = get_solution(GurobiModel)
	# println("soln = $(sol)")

	# objv = get_objval(GurobiModel)
	# println("objv = $(objv)")


	##### OSQP solver
	    
	# tic()
	# OSQPmdl = OSQP.Model() 	# needs SparseMatrixCSC,
	# A_osqp = sparse( [ Aeq_gurobi ; eye(N*n_uxu) ] )
	# lb_osqp = [ squeeze(beq_gurobi,2) ; squeeze(lb_gurobi,2) ]
	# ub_osqp = [ squeeze(beq_gurobi,2) ; squeeze(ub_gurobi,2) ]
	# P_osqp = sparse(2*H_gurobi)
	# OSQP.setup!(OSQPmdl; P=P_osqp, q=f_gurobi_init, A=A_osqp, l=lb_osqp, u=ub_osqp, verbose=0)
	# results_osqp = OSQP.solve!(OSQPmdl)
	# solv_time=toq()
	# println("1st solv time OSQP:  $(solv_time*1000) ms")
	


	# this function is called iteratively
	function solve_gurobi(s_0::Float64, v_0::Float64, u_0::Float64, s_ref::Array{Float64,1}, v_ref::Array{Float64,1})

		tic()


		# build problem
		x0 = [s_0 ; v_0]
		u0 = u_0 				# it's really u_{-1}
		x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

		# update RHS of linear equality constraint
		beq_gurobi_updated = repmat(g_tilde,N,1);
		beq_gurobi_updated[1:nx_tilde] = beq_gurobi_updated[1:nx_tilde] + A_tilde*x_tilde_0 	# PARAMETER: depends on x0

		# update reference trajectories
		x_ref = zeros(N*nx,1)
		for i = 1 : N
			x_ref[(i-1)*nx+1] = s_ref[i+1]		# set x_ref, s_ref/v_ref is of dim N+1
			x_ref[(i-1)*nx+2] = v_ref[i+1]		# set v_ref
		end
		# augment state with input for deltaU-formulation
		x_tilde_ref = zeros(N*(nx+nu))
		for i = 1 : N
			x_tilde_ref[(i-1)*(nx+nu)+1 : (i-1)*(nx+nu)+nx] = x_ref[(i-1)*nx+1 : (i-1)*nx+nx]
			x_tilde_ref[(i-1)*(nx+nu)+nx+1 : (i-1)*(nx+nu)+nx+nu] = u_ref_init[i]	# u_ref_init always 0, but no no weights
		end
		u_tilde_ref = zeros(N*nu) 	# want to minimize deltaU = u_k - u_{k-1}

		z_gurobi_ref = zeros(N*n_uxu) 	# reference point for z_gurobi ; PARAMETER!
		for i = 1 : N
			z_gurobi_ref[(i-1)*n_uxu+1 : (i-1)*n_uxu+nu] = u_tilde_ref[(i-1)*nu+1 : i*nu] 		# should be zero for this application
			z_gurobi_ref[(i-1)*n_uxu+nu+1 : i*n_uxu] = x_tilde_ref[(i-1)*(nx+nu)+1 : i*(nu+nx)]
		end 
		f_gurobi_updated = -2*H_gurobi*z_gurobi_ref


		# formulate optimization problem
		GurobiEnv = Gurobi.Env()	# really necessary?
		setparam!(GurobiEnv, "Presolve", -1)		# -1: automatic; no big influence on solution time
		setparam!(GurobiEnv, "LogToConsole", 0)		# set presolve to 0
		# setparam!(GurobiEnv, "TimeLimit",0.025)		# for 20Hz = 50ms
		# Formulate Optimization Problem
   	 	GurobiModel = gurobi_model(GurobiEnv;
    			name = "qp_01",
    			H = 2*H_gurobi,
    			f = f_gurobi_updated,	# need to make it "flat"
    			Aeq = Aeq_gurobi,
    			beq = squeeze(beq_gurobi_updated,2), # need to make it "flat"
    			lb = squeeze(lb_gurobi,2), # need to make it "flat"
    			ub = squeeze(ub_gurobi,2)	) # need to make it "flat"
	    optimize(GurobiModel)		 		# solve optimization problem
 		solvTimeGurobi1 = toq()
		optimizer_gurobi = get_solution(GurobiModel)
		status = get_status(GurobiModel)


###############################

		# OSQP implementation (no equality constraints)
		# min 1/2 * x' * P * x + q'*x
		# lb <= Ax <= ub
		
		# tic()
		# OSQPmdl = OSQP.Model() 	# needs SparseMatrixCSC,
		# lb_osqp_updated = [ squeeze(beq_gurobi_updated,2) ; squeeze(lb_gurobi,2) ]
		# ub_osqp_updated = [ squeeze(beq_gurobi_updated,2) ; squeeze(ub_gurobi,2) ]
		# OSQP.setup!(OSQPmdl; P=P_osqp, q=f_gurobi_updated, A=A_osqp, l=lb_osqp_updated, u=ub_osqp_updated, verbose=0)
		# # OSQP.update!(OSQPmdl; q=f_gurobi_updated, l=lb_osqp_updated, u=ub_osqp_updated)
		# results_osqp = OSQP.solve!(OSQPmdl)
		# solvTime_osqp = toq()

		# optimizer_gurobi = results_osqp.x
		# solvTimeGurobi1 = solvTime_osqp
		# status = results_osqp.info.status


###############################
		# alternatively, call via MathProgBase interface
		# http://mathprogbasejl.readthedocs.io/en/latest/quadprog.html
		# tic()
		# solution = quadprog(f_gurobi_updated, 2*H_gurobi, [Aeq_gurobi ; -Aeq_gurobi], '<', [squeeze(beq_gurobi_updated,2) ; -squeeze(beq_gurobi_updated,2)], squeeze(lb_gurobi,2), squeeze(ub_gurobi,2), GurobiSolver(Presolve=0, LogToConsole=0) )
		# solution = quadprog(f_gurobi_updated, 2*H_gurobi, [Aeq_gurobi ; -Aeq_gurobi], '<', [squeeze(beq_gurobi_updated,2) ; -squeeze(beq_gurobi_updated,2)], squeeze(lb_gurobi,2), squeeze(ub_gurobi,2), MosekSolver(LOG=0) )
		# solution = quadprog(f_gurobi_updated, 2*H_gurobi, [Aeq_gurobi ; -Aeq_gurobi], '<', [squeeze(beq_gurobi_updated,2) ; -squeeze(beq_gurobi_updated,2)], squeeze(lb_gurobi,2), squeeze(ub_gurobi,2), IpoptSolver(print_level=0) )
		# solvTimeGurobi2 = toq()
		# optimizer_gurobi = solution.sol

		# if solvTime_osqp > 1e-3
		# 	OSQPmdl = OSQP.Model() 	# needs SparseMatrixCSC,
		# 	lb_osqp_updated = [ squeeze(beq_gurobi_updated,2) ; squeeze(lb_gurobi,2) ]
		# 	ub_osqp_updated = [ squeeze(beq_gurobi_updated,2) ; squeeze(ub_gurobi,2) ]
		# 	OSQP.setup!(OSQPmdl; P=P_osqp, q=f_gurobi_updated, A=A_osqp, l=lb_osqp_updated, u=ub_osqp_updated, verbose=0)
		# 	# OSQP.update!(OSQPmdl; q=f_gurobi_updated, l=lb_osqp_updated, u=ub_osqp_updated)
		# 	tic()
		# 	results_osqp = OSQP.solve!(OSQPmdl)
		# 	solvTime_osqp1 = toq()
		# 	# pure solvTime very low; well under 5ms
		# 	# setup time can cause lots of trouble
		# 	println("*** old osqp pure solv time: $(solvTime_osqp) ***")		# 65ms (setup time)
		# 	println("*** new osqp pure solv time: $(solvTime_osqp1) ***")		# 0.4ms (setup time)
		# end

		# structure of z = [ (dAcc,s,v,Acc) ; (dAcc, s, v, Acc) ; ... ]
		a_pred_gurobi = optimizer_gurobi[4:n_uxu:end]
		s_pred_gurobi = [ s_0 ; optimizer_gurobi[2:n_uxu:end] ] 	# include current s 
		v_pred_gurobi = [ v_0 ; optimizer_gurobi[3:n_uxu:end] ]		# include current v 
		dA_pred_gurobi = optimizer_gurobi[1:n_uxu:end]				# include current v 
		
		# println("deltaA_gurobi: $(optimizer_gurobi[1:n_uxu:end]')")
		# println("s_pred_gurobi (incl s0): $(s_pred_gurobi)")
		# println("v_pred_gurobi (incl v0): $(v_pred_gurobi)")
		# println("a_pred_gurobi: $(a_pred_gurobi)")
		# println("a0_opt_gurobi: $(optimizer_gurobi[4])")

		acc_opt = optimizer_gurobi[4]

   	 	return acc_opt, a_pred_gurobi, s_pred_gurobi, v_pred_gurobi, dA_pred_gurobi, solvTimeGurobi1, status

	end  	# end of solve_gurobi()

end # end of module
