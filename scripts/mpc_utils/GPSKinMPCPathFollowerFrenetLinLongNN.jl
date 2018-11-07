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
## STATE CONSTRAINTS RELAXED



module GPSKinMPCPathFollowerFrenetLinLongNN
	__precompile__()

	using Gurobi
	using MAT


	import KinMPCParams 		# load basic parameters such as N, dt, L_a, L_b that is shared among the controllers

	println("Creating longitudinal kinematic bicycle model in NN...")
	println(pwd())

	if KinMPCParams.platform == "nuvo"
		primalNN_Data 	= matread("../GenesisAutoware/ros/src/genesis_path_follower/paths/goodNNs/trained_weightsPrimalLong1k_CPGDay3.mat")
		dualNN_Data 	= matread("../GenesisAutoware/ros/src/genesis_path_follower/paths/goodNNs/trained_weightsDualLong1k_CPGDay3.mat")
	elseif KinMPCParams.platform == "abby"
		primalNN_Data 	= matread("../catkin_ws/src/genesis_path_follower/paths/goodNNs/trained_weightsPrimalLong1k_CPGDay3.mat")
		dualNN_Data 	= matread("../catkin_ws/src/genesis_path_follower/paths/goodNNs/trained_weightsDualLong1k_CPGDay3.mat")
	else
		println("Lat NN Data not found!!!")
	end
	# read out NN primal weights
	Wi_PLong = primalNN_Data["W1"]
	bi_PLong = primalNN_Data["b1"]
	W1_PLong = primalNN_Data["W2"]
	b1_PLong = primalNN_Data["b2"]
	Wout_PLong = primalNN_Data["W0"]
	bout_PLong = primalNN_Data["b0"]

	Wi_DLong = dualNN_Data["W1"]
	bi_DLong = dualNN_Data["b1"]
	W1_DLong = dualNN_Data["W2"]
	b1_DLong = dualNN_Data["b2"]
	Wout_DLong = dualNN_Data["W0"]
	bout_DLong = dualNN_Data["b0"]

	# ====================== general problem formulation is given by ======================
	# x_{k+1} = A x_k + B u_k + g_k
	# u_lb <= u_k <= u_ub
	# x_lb <= x_k <= x_ub
	# dU_lb <= u_k - u_{k-1} <= dU_ub
	# minimize (x_k - x_k_ref)' * Q * (x_k - x_k_ref) + (u_k - u_k_ref)' * R * (u_k - u_k_ref) + (u_k - u_{k-1})' * Rdelta * (u_k - u_{k-1}) 

	# general control parameters
	# some might be passed on as arguments
	dt      = KinMPCParams.dt		# model discretization time, td (s)
	N       = KinMPCParams.N		# horizon, N=8 is standard
	L_a 	= KinMPCParams.L_a		# from CoG to front axle (according to Jongsang)
	L_b 	= KinMPCParams.L_b				# from CoG to rear axle (according to Jongsang)

	nx 		= KinMPCParams.nx_long				# dimension of x = (s,v)
	nu 		= KinMPCParams.nu_long				# number of inputs u = a

	# define System matrices (all time-invariant)
	A = [	1 	dt 		# can be made more exact using matrix exponential
			0	1 	]
	B = [ 	0
			dt 		]
	g = zeros(nx)

	# define cost functions
    C_s 	= KinMPCParams.C_s			# track progress
	C_v 	= KinMPCParams.C_v;			# ref velocity tracking weight			
	C_acc 	= KinMPCParams.C_acc
	C_dacc 	= KinMPCParams.C_dacc;		# 20 too high; 10 OK for med speed; 10 a big jerky for high speed; 13 too high

	Q = diagm([C_s ; C_v])	# create diagonal matrix
	R = C_acc
	Rdelta = C_dacc

	# define (box) constraints
	largeNumber = 1e5;		# use this number for variables that are not upper/lower bounded

	v_min = KinMPCParams.v_min		# vel bounds (m/s)
	v_max = KinMPCParams.v_max
	a_max = KinMPCParams.a_max 		# acceleration and deceleration bound, m/s^2
	a_dmax = KinMPCParams.a_dmax 	# jerk bound, m/s^3

	x_lb = [	-largeNumber	# make sure car doesnt travel more than largeNumber [m]
				v_min		]  	# v_min
	x_ub = [	largeNumber
				v_max		] 		# v_max

	u_lb = -a_max					# -a_max
	u_ub = a_max 					# a_max

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

	# println(ub_gurobi)
	# println(lb_gurobi)

	# build equality matrix (most MALAKA task ever)
	nu_tilde = nu
	nx_tilde = nu+nx
	# n_uxu = nu_tilde + nx_tilde
	Aeq_gurobi = zeros(N*nx_tilde , N*(nx_tilde+nu_tilde))
	Aeq_gurobi[1:nx_tilde, 1:(nx_tilde+nu_tilde)] = [-B_tilde eye(nx_tilde)] 	# fill out first row associated with x_tilde_1
	for i = 2 : N  	# fill out rows associated to x_tilde_2, ... , x_tilde_N
		Aeq_gurobi[ (i-1)*nx_tilde+1 : i*nx_tilde  , (i-2)*(nu_tilde+nx_tilde)+(nu_tilde)+1 : (i-2)*(nu_tilde+nx_tilde)+nu_tilde+(nx_tilde+nu_tilde+nx_tilde)    ] = [-A_tilde -B_tilde eye(nx_tilde)]
	end

	println(Aeq_gurobi)

	# right-hand-size of equality constraint
	beq_gurobi = repmat(g_tilde,N,1);
	beq_gurobi[1:nx_tilde] = beq_gurobi[1:nx_tilde] + A_tilde*x_tilde_0_init 	# PARAMETER: depends on x0

	println(beq_gurobi)

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
	println("1st solv time Gurobi LONG:  $(solv_time*1000) ms")

	# # access results
	# sol = get_solution(GurobiModel)
	# println("soln = $(sol)")

	# objv = get_objval(GurobiModel)
	# println("objv = $(objv)")

	# ================ Trafo 3 =================== 
	# used for NN implementation
	# notation: x_tilde = A_tilde_vec * x0 + B_tilde_vec * u_vec + E_tilde_vec * g_tilde 
	Q_tilde_vec = kron(eye(N),Q_tilde)   # for x_tilde_vec
	R_tilde_vec = kron(eye(N),R_tilde)	 # for u_tilde_vec

	A_tilde_vec = zeros(N*(nx+nu), (nx+nu))
	for ii = 1 : N
	    A_tilde_vec[1+(ii-1)*(nx+nu):ii*(nx+nu),:] = A_tilde^ii
	end

	B_tilde_vec = zeros(N*(nx+nu), nu*N)
	for ii = 0 : N-1
	    for jj = 0 : ii-1
	        B_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+jj*nu:  (jj+1)*nu] = A_tilde^(ii-jj)*B_tilde
	    end
	    B_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+ii*nu:(ii+1)*nu] = B_tilde
	end

	nw=nx+nu
	E_tilde_vec = zeros(N*(nx+nu), nw*N)

	for ii = 0 : N-1
	    for jj = 0 : ii-1
	        E_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+jj*nw:  (jj+1)*nw] = A_tilde^(ii-jj)*eye(nx+nu)
	    end
	    E_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+ii*nw:(ii+1)*nw] = eye(nx+nu)
	end
	g_tilde_vec = repmat(g_tilde,N)

	# build constraints
	Fu_tilde = [eye(nu) ; -eye(nu)]
	fu_tilde = [u_tilde_ub; -u_tilde_lb]
	ng = length(fu_tilde)
	# Concatenate input (tilde) constraints
	Fu_tilde_vec = kron(eye(N), Fu_tilde)
	fu_tilde_vec = repmat(fu_tilde,N)

	# Appended State constraints (tilde)
	# F_tilde = [eye(nx+nu) ; -eye(nx+nu)]
	# f_tilde = [x_tilde_ub ; -x_tilde_lb]
	# nf = length(f_tilde);
	nf = 0

	# Concatenate appended state (tilde) constraints
	# F_tilde_vec = kron(eye(N), F_tilde)
	# f_tilde_vec = repmat(f_tilde,N)   

	# these variables will be updated with current states
	x_tilde_ref = []

	Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
    # C_dual = [F_tilde_vec*B_tilde_vec; Fu_tilde_vec]		        # Adding state constraints 
    C_dual = Fu_tilde_vec		        # Adding state constraints 
    Qdual_tmp = C_dual*(Q_dual\(C_dual'))
    Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))

   
##################################################################################
##################################################################################
##################################################################################
	function eval_DualNN(params::Array{Float64,1})
		global x_tilde_ref

		tic()
		x_tilde_0 = [ 0; params[1:2] ]
		
		# some terms can be pre-computed
		c_dual = (2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*B_tilde_vec +
    	      - 2*x_tilde_ref'*Q_tilde_vec*B_tilde_vec)'

		const_dual = x_tilde_0'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0 + 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  + g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  - 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref - 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref +
                  + x_tilde_ref'*Q_tilde_vec*x_tilde_ref
        
	    # d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0 - F_tilde_vec*E_tilde_vec*g_tilde_vec;  fu_tilde_vec]
	    d_dual = fu_tilde_vec

   		# calls the NN with two Hidden Layers
		z1 = max.(Wi_DLong*params + bi_DLong, 0)
		z2 = max.(W1_DLong*z1 + b1_DLong, 0)
		lambda_tilde_NN_orig = Wout_DLong*z2 + bout_DLong
		lambda_tilde_NN_vec = max.(Wout_DLong*z2 + bout_DLong, 0)  	#Delta-Acceleration

		dualObj_NN = -1/2 * lambda_tilde_NN_vec'*Qdual_tmp*lambda_tilde_NN_vec - (C_dual*(Q_dual\c_dual)+d_dual)'*lambda_tilde_NN_vec - 1/2*c_dual'*(Q_dual\c_dual) + const_dual

		time_NN = toq()

		return dualObj_NN, lambda_tilde_NN_orig, time_NN
	end


	function eval_PrimalNN(params::Array{Float64,1})

		global x_tilde_ref 	# not sure if needed

		# s_0 = params[1]
		# v_0 = params[2]
		s_0 = 0
		v_0 = params[1]

		tic()

		# calls the NN with two Hidden Layers
		z1 = max.(Wi_PLong*params  + bi_PLong, 0)
		z2 = max.(W1_PLong*z1      + b1_PLong, 0)
		u_tilde_NN_vec = Wout_PLong*z2 + bout_PLong  	#Delta-Acceleration

		# project
		u_tilde_NN_vec = min.(u_tilde_NN_vec, u_tilde_ub)
		u_tilde_NN_vec = max.(u_tilde_NN_vec, u_tilde_lb)

		# compute NN predicted state
		x_tilde_0 = [ 0; params[1:2] ] 	
		x_tilde_NN_vec = A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_NN_vec + E_tilde_vec*g_tilde_vec

		## verify feasibility
		# xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
		xu_tilde_NN_res = maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec)

		flag_XUfeas = 0
		if maximum(xu_tilde_NN_res) <= 1e-5  	# infeasible if bigger than zero/threshold
			flag_XUfeas = 1
		end

		## check optimality (compute objective value) ##
		# println("11111111")
		primObj_NN = (x_tilde_NN_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_NN_vec-x_tilde_ref) + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec
		# println("primObj_NN: $(primObj_NN)")
		# println("primObj_NN: $(size(primObj_NN)")

		solvTime_NN = toq()

		a_opt_NN = 	x_tilde_NN_vec[3]
		a_pred_NN = x_tilde_NN_vec[3:(nx+nu):end]
		s_pred_NN = [ s_0 ; x_tilde_NN_vec[1:(nx+nu):end] ]
		v_pred_NN = [ v_0 ; x_tilde_NN_vec[2:(nx+nu):end] ]
		# dA_pred_NN = u_tilde_NN_vec

		# println(size(u_tilde_NN_vec))
		# println(size(squeeze(u_tilde_NN_vec,1)))
		# println(size(squeeze(u_tilde_NN_vec,2)))

		return primObj_NN, xu_tilde_NN_res, flag_XUfeas, a_opt_NN, a_pred_NN, s_pred_NN, v_pred_NN, squeeze(u_tilde_NN_vec,2), solvTime_NN
		
	end


	# only reference functions need to be updated
	function updateMatrices(s_ref::Array{Float64,1}, v_ref::Array{Float64,1})

		global x_tilde_ref

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

	end


	function get_NNsolution(s_0::Float64, v_0::Float64, u_0::Float64, s_ref::Array{Float64,1}, v_ref::Array{Float64,1})

		# try to make s_ref smaller
		s_ref_true = s_ref
		s_0_true = s_0

		s_ref = s_ref - s_0
		s_0 = 0.0

		# println("s_0: $(s_0)")
		# println(typeof(s_0))

		updateMatrices(s_ref, v_ref)

		# stack everything together
		# params = [s_0 ; v_0 ; u_0 ; s_ref[2:end] ; v_ref[2:end]] 	# stack to 19x1 matrix
		params = [v_0 ; u_0 ; s_ref[2:end] ; v_ref[2:end]] 	# stack to 19x1 matrix

		# eval NN solution
		primNN_obj, xu_tilde_NN_res, flag_XUfeas, a_opt_NN, a_pred_NN, s_pred_NN, v_pred_NN, dA_pred_NN, solvTime_primNN = eval_PrimalNN(params)
		dualNN_obj, lambda_tilde_NN_vec, solvTime_dualNN = eval_DualNN(params)

		# println("primal NN obj: $(primNN_obj)")
		# println("dual NN obj: $(dualNN_obj)")


		is_opt_NN = (flag_XUfeas==1) && (primNN_obj[1] - dualNN_obj[1] <= 0.1)
		is_opt_NN = (flag_XUfeas==1)

		# is_opt_NN = false 	# always use GUROBI

		if is_opt_NN
			# println("****** IS FEASIBLE: $(is_opt_NN) ******")
			solMode = "NN"
			# needs a bit of work
			# a_opt_NN, a_pred_NN, s_pred_NN, v_pred_NN, dA_pred_NN, solvTime_NN, is_opt_NN = solve_gurobi(s_0, v_0, u_0, s_ref, v_ref)
			return 	a_opt_NN, a_pred_NN, s_pred_NN+s_0_true, v_pred_NN, dA_pred_NN, solvTime_primNN + solvTime_dualNN, is_opt_NN, solMode, primNN_obj, dualNN_obj,xu_tilde_NN_res

		else  	## NN solution not good
			solMode = "opt"
			primNN_obj = []
			dualNN_obj = []
			xu_tilde_NN_res = []
			a_opt_gurobi, a_pred_gurobi, s_pred_gurobi, v_pred_gurobi, dA_pred_gurobi, solv_time_long_gurobi1, is_opt_long = solve_gurobi(s_0, v_0, u_0, s_ref, v_ref)
			return a_opt_gurobi, a_pred_gurobi, s_pred_gurobi+s_0_true, v_pred_gurobi, dA_pred_gurobi, solv_time_long_gurobi1, is_opt_long, solMode, primNN_obj, dualNN_obj,xu_tilde_NN_res
		end 

		################ BACKUP ################
		# return a_opt_gurobi, a_pred_gurobi, s_pred_gurobi, v_pred_gurobi, dA_pred_gurobi, solv_time_long_gurobi1, is_opt_long
	end

	# this function is called iteratively
	function solve_gurobi(s_0::Float64, v_0::Float64, u_0::Float64, s_ref::Array{Float64,1}, v_ref::Array{Float64,1})

		global x_tilde_ref

		tic()

		# to modify variables, need global
		# global v_max
		# println(v_max)
		# v_max = randn(1)
		# println(v_max)
		

		# build problem
		# tmp fix to make s not go weird

		x0 = [s_0 ; v_0]
		u0 = u_0 				# it's really u_{-1}
		x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

		# update RHS of linear equality constraint
		beq_gurobi_updated = repmat(g_tilde,N,1);
		beq_gurobi_updated[1:nx_tilde] = beq_gurobi_updated[1:nx_tilde] + A_tilde*x_tilde_0 	# PARAMETER: depends on x0

		# # update reference trajectories
		# x_ref = zeros(N*nx,1)
		# for i = 1 : N
		# 	x_ref[(i-1)*nx+1] = s_ref[i+1]		# set x_ref, s_ref/v_ref is of dim N+1
		# 	x_ref[(i-1)*nx+2] = v_ref[i+1]		# set v_ref
		# end
		# # augment state with input for deltaU-formulation
		# x_tilde_ref = zeros(N*(nx+nu))
		# for i = 1 : N
		# 	x_tilde_ref[(i-1)*(nx+nu)+1 : (i-1)*(nx+nu)+nx] = x_ref[(i-1)*nx+1 : (i-1)*nx+nx]
		# 	x_tilde_ref[(i-1)*(nx+nu)+nx+1 : (i-1)*(nx+nu)+nx+nu] = u_ref_init[i]	# u_ref_init always 0, but no no weights
		# end

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
