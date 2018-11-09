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
		- this module is concerned with lateral control
		- tracks (e_y, e_psi)

=#




module GPSKinMPCPathFollowerFrenetLinLatNN
	__precompile__()

	using Gurobi
	using MAT

	import KinMPCParams 		# load basic parameters such as N, dt, L_a, L_b that is shared among the controllers

	println("Creating lateral kinematic bicycle model in NN ....")
	println(pwd())

	# saved place is different
	if KinMPCParams.platform == "nuvo"
		primalNN_Data 	= matread("../GenesisAutoware/ros/src/genesis_path_follower/paths/trained_weightsPrimalLatBadRand10kData_CPGDay5.mat")
		# primalNN_Data 	= matread("../GenesisAutoware/ros/src/genesis_path_follower/paths/goodNNs/trained_weightsPrimalLat10k_CPGDay2BacktoDay1Tune.mat")
		dualNN_Data 	= matread("../GenesisAutoware/ros/src/genesis_path_follower/paths/trained_weightsDualLat.mat")

	elseif KinMPCParams.platform == "abby"
		primalNN_Data 	= matread("../catkin_ws/src/genesis_path_follower/paths/trained_weightsPrimalLatBadRand10kData_CPGDay5.mat")
		# primalNN_Data 	= matread("../catkin_ws/src/genesis_path_follower/paths/goodNNs/trained_weightsPrimalLat10k_CPGDay2BacktoDay1Tune.mat")
		dualNN_Data 	= matread("../catkin_ws/src/genesis_path_follower/paths/trained_weightsDualLat.mat")
	
	else
		println("Long NN Data not found!")
	end

	println(keys(primalNN_Data))

	# read out NN primal weights
	Wi_PLat = primalNN_Data["W1"]
	bi_PLat = primalNN_Data["b1"]
	W1_PLat = primalNN_Data["W2"]
	b1_PLat = primalNN_Data["b2"]
	Wout_PLat = primalNN_Data["W0"]
	bout_PLat = primalNN_Data["b0"]

	Wi_DLat = dualNN_Data["W1D"]
	bi_DLat = dualNN_Data["b1D"]
	W1_DLat = dualNN_Data["W2D"]
	b1_DLat = dualNN_Data["b2D"]
	Wout_DLat = dualNN_Data["W0D"]
	bout_DLat = dualNN_Data["b0D"]


	# ====================== general problem formulation is given by ======================
	# x_{k+1} = A_k x_k + B_k u_k + g_k
	# u_lb <= u_k <= u_ub
	# x_lb <= x_k <= x_ub
	# dU_lb <= u_k - u_{k-1} <= dU_ub
	# minimize (x_k - x_k_ref)' * Q * (x_k - x_k_ref) + (u_k - u_k_ref)' * R * (u_k - u_k_ref) + (u_k - u_{k-1})' * Rdelta * (u_k - u_{k-1}) 
	# x = (ey, epsi), u = (df)

	# general control parameters
	dt      = KinMPCParams.dt		# model discretization time, td (s)
	N       = KinMPCParams.N		# horizon, N=8 is standard
	L_a 	= KinMPCParams.L_a		# from CoG to front axle (according to Jongsang)
	L_b 	= KinMPCParams.L_b				# from CoG to rear axle (according to Jongsang)

	nx 		= KinMPCParams.nx_lat				# dimension of x = (ey,epsi)
	nu 		= KinMPCParams.nu_lat				# number of inputs u = df

	# predicted (s,v); "init" for initial solve
    s_pred_init   = 1.0*collect(0:dt:(N)*dt)		# predicted s(0), s(1), ..., s(N)
	v_pred_init = ones(N+1,1)						# predicted v(0), v(1), ..., v(N)
	k_coeff_init = [0.0, 0.0, 0.0, 0.0]		# K(s) = a0 + a1 * s + a2 * s^2 + a3 * s^3

	# system dynamics A, B, c defined later on (they depend on reference)
	A_init = zeros(nx, nx, N)
	B_init = zeros(nx, nu, N)
	g_init = zeros(nx, N)
	for i = 1 : N
		A_init[:,:,i] = [	1	dt*v_pred_init[i] 
							0		1			]
		B_init[:,:,i] = [	dt*v_pred_init[i]*L_b/(L_a+L_b) 
							dt*v_pred_init[i] / (L_a + L_b)	]
		g_init[:,i] = [ 0	# column vector
						-dt*v_pred_init[i]*(k_coeff_init[1]*s_pred_init[i]^3 + k_coeff_init[2]*s_pred_init[i]^2 + k_coeff_init[3]*s_pred_init[i] + k_coeff_init[4]) 	]
	end

	# define cost functions
    C_ey 	= KinMPCParams.C_ey				# lateral deviation
    C_epsi 	= KinMPCParams.C_epsi			# heading deviation
	C_df	= KinMPCParams.C_df				# 150	# tire angle input
	C_ddf	= KinMPCParams.C_ddf			# 3e4			# derivative of tire angle input

	Q = diagm([C_ey ; C_epsi])	# state cost
	R = C_df 					# input cost
	Rdelta = C_ddf				# deltaU cost

	# define (box) constraints
	largeNumber = 1e2;		# use this number for variables that are not upper/lower bounded
	ey_min = -largeNumber
	ey_max = largeNumber
	epsi_min = -largeNumber
	epsi_max = largeNumber

	df_max = KinMPCParams.df_max		# steering
	ddf_max = KinMPCParams.ddf_max		# change in steering

	# collect constraint
	x_lb = [	ey_min
				epsi_min	]
	x_ub = [	ey_max
				epsi_max 	]
	u_lb = -df_max
	u_ub = df_max

	dU_lb = -ddf_max*dt
	dU_ub = ddf_max*dt


	# build references (which are updated)
	# get init x_ref for one initial solve (to speed up Julia)
	ey_ref_init = zeros(N) 		# regulate ey to zero
	epsi_ref_init = zeros(N)	# regulate epsi to zero
	x_ref_init = zeros(N*nx)
	for i = 1 : N
		x_ref_init[(i-1)*nx+1] = ey_ref_init[i]		# set x_ref
		x_ref_init[(i-1)*nx+2] = epsi_ref_init[i]		# set v_ref
	end	

	# input reference
	u_ref_init = zeros(N,1)	# if not used, set cost to zeros

	# get Initial state and input
	# should be done dynamically later on
	ey0_init = 0
	epsi0_init = 0
	x0_init = [ey0_init ; epsi0_init]
	u0_init = 0

	# ================== Transformation 1 ======================
	# augment state and redefine system dynamics (A,B,g) and constraints
	# x_tilde_k := (x_k , u_{k-1})
	# u_tilde_k := (u_k - u_{k-1})
	A_tilde_init = zeros(nx+nu,nx+nu,N)
	B_tilde_init = zeros(nx+nu,nu,N)
	g_tilde_init = zeros(nx+nu,N)
	for i = 1 : N 
		A_tilde_init[:,:,i] = [ A_init[:,:,i]  		B_init[:,:,i] 
								zeros(nu,nx)   		eye(nu)			]
		B_tilde_init[:,:,i] = [	B_init[:,:,i] 	;  	eye(nu)	]
		g_tilde_init[:,i] =   [	g_init[:,i]		; 	zeros(nu) ]
	end

	x_tilde_lb = [x_lb ; u_lb]
	x_tilde_ub = [x_ub ; u_ub]
	u_tilde_lb = dU_lb
	u_tilde_ub = dU_ub

	Q_tilde = [	Q 				zeros(nx,nu)
				zeros(nu,nx)	R 				]

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
	H_block = [	R_tilde 			zeros(nu, nu+nx)
				zeros(nu+nx,nu) 	Q_tilde				]
	H_gurobi = kron(eye(N), H_block)

	z_gurobi_ref_init = zeros(N*n_uxu) 	# reference point for z_gurobi ; PARAMETER!
	for i = 1 : N
		z_gurobi_ref_init[(i-1)*n_uxu+1 : (i-1)*n_uxu+nu] = u_tilde_ref_init[(i-1)*nu+1 : i*nu] 		# should be zero for this application
		z_gurobi_ref_init[(i-1)*n_uxu+nu+1 : i*n_uxu] = x_tilde_ref_init[(i-1)*(nx+nu)+1 : i*(nu+nx)]
	end 

	f_gurobi_init = -2*H_gurobi*z_gurobi_ref_init

	# build box constraints lb_gurobi <= z <= ub_gurobi
	# recall: z = (u_tilde, x_tilde, ....)
	lb_gurobi = repmat([u_tilde_lb ; x_tilde_lb], N)		# (deltaU, X, U)
	ub_gurobi = repmat([u_tilde_ub ; x_tilde_ub], N)		# (deltaU, X, U)

	# build equality matrix (most MALAKA task ever)
	nu_tilde = nu
	nx_tilde = nu+nx
	# n_uxu = nu_tilde + nx_tilde
	Aeq_gurobi_init = zeros(N*nx_tilde , N*(nx_tilde+nu_tilde))
	Aeq_gurobi_init[1:nx_tilde, 1:(nx_tilde+nu_tilde)] = [-B_tilde_init[:,:,1] eye(nx_tilde)] 	# fill out first row associated with x_tilde_1
	for i = 2 : N  	# fill out rows associated to x_tilde_2, ... , x_tilde_N
		Aeq_gurobi_init[ (i-1)*nx_tilde+1 : i*nx_tilde  , (i-2)*(nu_tilde+nx_tilde)+(nu_tilde)+1 : (i-2)*(nu_tilde+nx_tilde)+nu_tilde+(nx_tilde+nu_tilde+nx_tilde)    ] = [-A_tilde_init[:,:,i] -B_tilde_init[:,:,i] eye(nx_tilde)]
	end

	# right-hand-size of equality constraint
	beq_gurobi_init = zeros(N*nx_tilde)
	for i = 1 : N
		beq_gurobi_init[(i-1)*nx_tilde+1:i*nx_tilde] = g_tilde_init[:,i]
	end
	beq_gurobi_init[1:nx_tilde] = beq_gurobi_init[1:nx_tilde] + A_tilde_init[:,:,1]*x_tilde_0_init 	# PARAMETER: depends on x0

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
    			Aeq = Aeq_gurobi_init,
    			beq = beq_gurobi_init,	# PARAMETER that depends on x0, u_{-1}	
    			lb = lb_gurobi,
    			ub = ub_gurobi		)
    optimize(GurobiModel)
	solv_time=toq()
	println("1st solv time Gurobi:  $(solv_time*1000) ms")

	## access results
	# sol = get_solution(GurobiModel)
	# println("soln = $(sol)")

	# objv = get_objval(GurobiModel)
	# println("objv = $(objv)")



	# ================ Trafo 3 =================== 
	Q_tilde_vec = kron(eye(N),Q_tilde)   # for x_tilde_vec
	R_tilde_vec = kron(eye(N),R_tilde)	 # for u_tilde_vec

	# A_tilde_vec, B_tilde_vec, E_tilde_vec, g_tilde_vec updated below

	Fu_tilde = [eye(nu) ; -eye(nu)]
	fu_tilde = [u_tilde_ub; -u_tilde_lb]
	ng = length(fu_tilde)

	# Concatenate input (tilde) constraints
	Fu_tilde_vec = kron(eye(N), Fu_tilde)
	fu_tilde_vec = repmat(fu_tilde,N)

	# Appended State constraints (tilde)
	F_tilde = [eye(nx+nu) ; -eye(nx+nu)]
	f_tilde = [x_tilde_ub ; -x_tilde_lb]
	nf = length(f_tilde);

	# Concatenate appended state (tilde) constraints
	F_tilde_vec = kron(eye(N), F_tilde)
	f_tilde_vec = repmat(f_tilde,N)   

	NNgapThreshold_lat = KinMPCParams.NNgapThreshold_lat

	# will be updated below
	x_tilde_ref = []
	A_tilde_updated = []
	B_tilde_updated = []
	g_tilde_updated = []

	c_pred 		= []

######################################### main functions #######################################



	function updateMatrices(s_pred, v_pred, k_coeffs)
		global A_tilde_updated, B_tilde_updated, g_tilde_updated, c_pred

		# prepare data for Lat control
		
		# system dynamics A, B, c defined later on (they depend on reference)
		A_updated = zeros(nx, nx, N)
		B_updated = zeros(nx, nu, N)
		g_updated = zeros(nx, N)
		c_pred = zeros(N)

		for i = 1 : N
			c_pred[i] = k_coeffs[1]*s_pred[i]^3 + k_coeffs[2]*s_pred[i]^2 + k_coeffs[3]*s_pred[i] + k_coeffs[4]
			A_updated[:,:,i] = [	1	dt*v_pred[i] 
									0		1			]
			B_updated[:,:,i] = [	dt*v_pred[i]*L_b/(L_a+L_b) 
									dt*v_pred[i]/(L_a + L_b)	]
			g_updated[:,i] = [ 0	# column vector
							-dt*v_pred[i]*c_pred[i] 	]
		end


		# x_tilde transformation
		# update system matrices for tilde-notation
		A_tilde_updated = zeros(nx+nu,nx+nu,N)
		B_tilde_updated = zeros(nx+nu,nu,N)
		g_tilde_updated = zeros(nx+nu,N)
		for i = 1 : N 
			A_tilde_updated[:,:,i] = [ 	A_updated[:,:,i]  		B_updated[:,:,i] 
										zeros(nu,nx)   			eye(nu)			]
			B_tilde_updated[:,:,i] = [	B_updated[:,:,i] 	;  	eye(nu)	]
			g_tilde_updated[:,i] =   [	g_updated[:,i]		; 	zeros(nu) ]
		end

	end


	function eval_PrimalNN(params::Array{Float64,1})

		global A_tilde_updated, B_tilde_updated, g_tilde_updated

		ey_0 = params[1]
		epsi_0 = params[2]

		tic()
		z1 = max.( Wi_PLat*params + bi_PLat, 0  )
		z2 = max.( W1_PLat*z1     + b1_PLat, 0  )
		u_tilde_NN_vec = Wout_PLat*z2 + bout_PLat


		x_tilde_0 = params[1:3] 	

		A_tilde_vec = zeros(N*(nx+nu), (nx+nu))
		A_tmp = eye(nx+nu)  	# tmp variable used to store the ``powers of A_tilde"
		for ii = 1 : N
			A_tmp = A_tilde_updated[:,:,ii]*A_tmp
		    A_tilde_vec[1+(ii-1)*(nx+nu):ii*(nx+nu),:] = A_tmp 	#A_tilde^ii
		end

		B_tilde_vec = zeros(N*(nx+nu), nu*N)
		for ii = 0 : N-1
		    for jj = 0 : ii-1
		    	A_tmp = eye(nx+nu)	# used to emulate A_tilde^(ii-jj)
		    	for kk = 1 : (ii-jj)
		    		A_tmp = A_tilde_updated[:,:,kk+1]*A_tmp
		    	end
		        B_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+jj*nu:  (jj+1)*nu] = A_tmp*B_tilde_updated[:,:,jj+1] 	# A_tilde^(ii-jj)*B_tilde
		    end
		    B_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+ii*nu:(ii+1)*nu] = B_tilde_updated[:,:,ii+1]
		end

		nw=nx+nu
		E_tilde_vec = zeros(N*(nx+nu), nw*N)
		for ii = 0 : N-1
		    for jj = 0 : ii-1
		    	A_tmp = eye(nx+nu) 	# simulates A_tilde^(ii-jj)
		    	for kk = 1 : (ii-jj)
		    		A_tmp = A_tilde_updated[:,:,kk+1]*A_tmp
		    	end
		        E_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+jj*nw:  (jj+1)*nw] = A_tmp * eye(nx+nu)    # A_tilde^(ii-jj)*eye(nx+nu)
		    end
		    E_tilde_vec[1+ii*(nx+nu):(ii+1)*(nx+nu), 1+ii*nw:(ii+1)*nw] = eye(nx+nu)
		end

		g_tilde_vec = zeros(N*(nx+nu))
		for ii = 1 : N
			g_tilde_vec[1+(ii-1)*(nx+nu) : ii*(nx+nu)] = g_tilde_updated[:,ii]
		end

		x_tilde_ref = x_tilde_ref_init
		x_tilde_NN_vec = A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_NN_vec + E_tilde_vec*g_tilde_vec

		xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
		

		flag_XUfeas = 0
		if maximum(xu_tilde_NN_res) <= 0  	# infeasible if bigger than zero/threshold
			flag_XUfeas = 1
		end


		primObj_NN = (x_tilde_NN_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_NN_vec-x_tilde_ref) + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec

		time_NN = toq()

		df_opt_NN = x_tilde_NN_vec[3]
		df_pred_NN = x_tilde_NN_vec[3:(nx+nu):end]
		ey_pred_NN = [ ey_0 ; x_tilde_NN_vec[1:(nx+nu):end] ]
		epsi_pred_NN = [ epsi_0 ; x_tilde_NN_vec[2:(nx+nu):end] ]


		return primObj_NN, xu_tilde_NN_res, flag_XUfeas, df_opt_NN, df_pred_NN, ey_pred_NN, epsi_pred_NN, squeeze(u_tilde_NN_vec,2), time_NN 
	end



	function get_NNsolution(ey_0::Float64, epsi_0::Float64, u_0::Float64, s_pred::Array{Float64,1}, v_pred::Array{Float64,1}, k_coeffs::Array{Float64,1})

		global c_pred

		updateMatrices(s_pred, v_pred, k_coeffs)

		params = [ey_0 ; epsi_0 ; u_0 ; v_pred[1:N] ; c_pred]

		# println("isze of params: $(size(params))")

		primNN_obj, xu_tilde_NN_res, flag_XUfeas, df_opt_NN, df_pred_NN, ey_pred_NN, epsi_pred_NN, ddf_pred_NN, solvTime_primNN = eval_PrimalNN(params)

		solvTime_dualNN = 0
		dualNN_obj = -Inf

		# is_opt_NN = (flag_XUfeas==1) && (primNN_obj[1] - dualNN_obj[1] <= NNgapThreshold_lat)
		is_opt_NN = (flag_XUfeas==1)
		# is_opt_NN = false

		if is_opt_NN
			println("****** LAT IS FEASIBLE: $(is_opt_NN) ******")
			solMode = "NN"
			# needs a bit of work
			# df_opt_NN, df_pred_NN, ddf_pred_NN, ey_pred_NN, epsi_pred_NN, solvTime_primNN, is_opt_NN = solve_gurobi(ey_0, epsi_0, u_0, s_pred, v_pred, k_coeffs)
			return 	df_opt_NN, df_pred_NN, ey_pred_NN, epsi_pred_NN, ddf_pred_NN, solvTime_primNN + solvTime_dualNN, is_opt_NN, solMode, primNN_obj, dualNN_obj,xu_tilde_NN_res
		else  	## NN solution not good
			solMode = "opt"
			# primNN_obj = []
			# dualNN_obj = []
			# xu_tilde_NN_res = []
			df_opt_gurobi, df_pred_gurobi, ddf_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, solvTimeGurobi1, is_opt_lat = solve_gurobi(ey_0, epsi_0, u_0, s_pred, v_pred, k_coeffs)
			return df_opt_gurobi, df_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, ddf_pred_gurobi, solvTimeGurobi1, is_opt_lat, solMode, primNN_obj, dualNN_obj,xu_tilde_NN_res
		end 

		# return df_opt, df_pred_gurobi, ddf_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, solvTimeGurobi1, status
	end


	# this function is called iteratively
	# may want to add epsi_ref in the future (epsi in a curve is generally non-zero)
	function solve_gurobi(ey_0::Float64, epsi_0::Float64, u_0::Float64, s_pred::Array{Float64,1}, v_pred::Array{Float64,1}, k_coeffs::Array{Float64,1})
		
		tic()

		global A_tilde_updated, B_tilde_updated, g_tilde_updated


		updateMatrices(s_pred, v_pred, k_coeffs)


		# build problem
		x0 = [ey_0 ; epsi_0]
		u0 = u_0 				# it's really u_{-1}
		x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER


		# # system dynamics A, B, c defined later on (they depend on reference)
		# A_updated = zeros(nx, nx, N)
		# B_updated = zeros(nx, nu, N)
		# g_updated = zeros(nx, N)
		# for i = 1 : N
		# 	A_updated[:,:,i] = [	1	dt*v_pred[i] 
		# 							0		1			]
		# 	B_updated[:,:,i] = [	dt*v_pred[i]*L_b/(L_a+L_b) 
		# 							dt*v_pred[i]/(L_a + L_b)	]
		# 	g_updated[:,i] = [ 0	# column vector
		# 					-dt*v_pred[i]*(k_coeffs[1]*s_pred[i]^3 + k_coeffs[2]*s_pred[i]^2 + k_coeffs[3]*s_pred[i] + k_coeffs[4]) 	]
		# end


		# # x_tilde transformation
		# # update system matrices for tilde-notation
		# A_tilde_updated = zeros(nx+nu,nx+nu,N)
		# B_tilde_updated = zeros(nx+nu,nu,N)
		# g_tilde_updated = zeros(nx+nu,N)
		# for i = 1 : N 
		# 	A_tilde_updated[:,:,i] = [ 	A_updated[:,:,i]  		B_updated[:,:,i] 
		# 								zeros(nu,nx)   			eye(nu)			]
		# 	B_tilde_updated[:,:,i] = [	B_updated[:,:,i] 	;  	eye(nu)	]
		# 	g_tilde_updated[:,i] =   [	g_updated[:,i]		; 	zeros(nu) ]
		# end


		# z-transformation
		Aeq_gurobi_updated = zeros(N*nx_tilde , N*(nx_tilde+nu_tilde))
		Aeq_gurobi_updated[1:nx_tilde, 1:(nx_tilde+nu_tilde)] = [-B_tilde_updated[:,:,1] eye(nx_tilde)] 	# fill out first row associated with x_tilde_1
		for i = 2 : N  	# fill out rows associated to x_tilde_2, ... , x_tilde_N
			Aeq_gurobi_updated[ (i-1)*nx_tilde+1 : i*nx_tilde  , (i-2)*(nu_tilde+nx_tilde)+(nu_tilde)+1 : (i-2)*(nu_tilde+nx_tilde)+nu_tilde+(nx_tilde+nu_tilde+nx_tilde)    ] = [-A_tilde_updated[:,:,i] -B_tilde_updated[:,:,i] eye(nx_tilde)]
		end

		# right-hand-size of equality constraint
		beq_gurobi_updated = zeros(N*nx_tilde)
		for i = 1 : N
			beq_gurobi_updated[(i-1)*nx_tilde+1:i*nx_tilde] = g_tilde_updated[:,i]
		end
		beq_gurobi_updated[1:nx_tilde] = beq_gurobi_updated[1:nx_tilde] + A_tilde_updated[:,:,1]*x_tilde_0 	# PARAMETER: depends on x0

		# FUTURE: implement references

		# println("H_gurobi:  $(H_gurobi)")
		# println("f_gurobi_init:  $(f_gurobi_init)")

	    GurobiEnv = Gurobi.Env()
		setparam!(GurobiEnv, "Presolve", -1)	# -1: automatic; example has 0; no big influence on solution time
		setparam!(GurobiEnv, "LogToConsole", 0)	# # set presolve to 0 what does it mean?
		# setparam!(GurobiEnv, "TimeLimit",0.025)		# for 20Hz = 50ms

		# note that: (1/2) * z' * (2*H) * z + f' * z
	    GurobiModel = gurobi_model(GurobiEnv;
	    			name = "qp_01",
	    			H = 2*H_gurobi,
	    			f = f_gurobi_init,	# PARAMETER that depends on x_ref and u_ref
	    			Aeq = Aeq_gurobi_updated,
	    			beq = beq_gurobi_updated,	# PARAMETER that depends on x0, u_{-1}	
	    			lb = lb_gurobi,
	    			ub = ub_gurobi		)
	    optimize(GurobiModel)
	 	solvTimeGurobi1 = toq()
		optimizer_gurobi = get_solution(GurobiModel)
		status = get_status(GurobiModel)

		# tic()
		# OSQPmdl = OSQP.Model() 	# needs SparseMatrixCSC,
		# lb_osqp_updated = [ beq_gurobi_updated ; lb_gurobi ]
		# ub_osqp_updated = [ beq_gurobi_updated ; ub_gurobi ]
		# A_osqp_updated = sparse( [ Aeq_gurobi_updated ; eye(N*n_uxu) ] )
		# OSQP.setup!(OSQPmdl; P=P_osqp, q=f_gurobi_init, A=A_osqp_updated, l=lb_osqp_updated, u=ub_osqp_updated, verbose=0)
		# # OSQP.update!(OSQPmdl; q=f_gurobi_updated, l=lb_osqp_updated, u=ub_osqp_updated)
		# results_osqp = OSQP.solve!(OSQPmdl)
		# solvTime_osqp = toq()

		# optimizer_gurobi = results_osqp.x
		# solvTimeGurobi1 = solvTime_osqp
		# status = results_osqp.info.status


  #       println("---- ey0, epsi0 in Gurobi:  $(x0') ---- ")
  #       println("u0 in Gurobi: $(u_0)")

		# println("s_pred in Gurobi (incl s0): $(s_pred)")
		# println("v_pred in Gurobi (incl v0): $(v_pred)")
		# println("k_coeffs in Gurobi: $(k_coeffs)")
		# @printf "%.20f \n"  k_coeffs[1]
		# @printf "%.20f \n"  k_coeffs[2]
		# @printf "%.20f \n"  k_coeffs[3]
		# @printf "%.20f \n"  k_coeffs[4]

		# log_str_lat = @sprintf("Solve Status Lat. IPOPT: %s, SA: %.3f, SolvTimeLat:  %.3f", is_opt_lat, df_opt, solv_time_lat)
		# loginfo(log_str_lat)


		# structure of z = [ (ddf,ey,epsi,df) ; (ddf, ey, epsi, df) ; ... ]
		ddf_pred_gurobi = optimizer_gurobi[1:n_uxu:end]
		df_pred_gurobi = optimizer_gurobi[4:n_uxu:end]
		ey_pred_gurobi = [ ey_0 ; optimizer_gurobi[2:n_uxu:end] ] 	# include current s 
		epsi_pred_gurobi = [ epsi_0 ; optimizer_gurobi[3:n_uxu:end] ]		# include current v 

		# println("ddf_pred_gurobi: $(ddf_pred_gurobi)")
		# println("df_pred_gurobi: $(df_pred_gurobi)")
		# println("ey_pred_gurobi (incl ey0): $(ey_pred_gurobi)")
		# println("epsi_pred_gurobi (incl v0): $(epsi_pred_gurobi)")

		df_opt = optimizer_gurobi[4]
	   	return df_opt, df_pred_gurobi, ddf_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, solvTimeGurobi1, status
	   	# return df_opt, df_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi,  solvTimeGurobi1, status

	end  	# end of solve_gurobi()

end
