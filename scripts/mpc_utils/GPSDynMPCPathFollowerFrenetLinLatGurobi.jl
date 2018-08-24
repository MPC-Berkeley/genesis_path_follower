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



module GPSDynMPCPathFollowerFrenetLinLatGurobi
	__precompile__()

	using Gurobi

	println("Creating lateral dynamic bicycle model in Gurobi/OSQP ....")


	# ====================== general problem formulation is given by ======================
	# x_{k+1} = A_k x_k + B_k u_k + g_k
	# u_lb <= u_k <= u_ub
	# x_lb <= x_k <= x_ub
	# dU_lb <= u_k - u_{k-1} <= dU_ub
	# minimize (x_k - x_k_ref)' * Q * (x_k - x_k_ref) + (u_k - u_k_ref)' * R * (u_k - u_k_ref) + (u_k - u_{k-1})' * Rdelta * (u_k - u_{k-1}) 
	#
	# x = (ey, epsi, vy, wz), u = (df)

	# general control parameters
	#dt      = 0.20			# model discretization time, td (s)
	dt 		= 0.1
	#N       = 8				# horizon
	N 		= 16
	nx 		= 4				# dimension of x = (ey,epsi,vy,wz)
	nu 		= 1				# number of inputs u = df
	# L_a     = 1.108 		# dist from CoG to front axle (m)
	L_a 	= 1.5213		# from CoG to front axle (according to Jongsang)
	# L_b     = 1.742 		# dist from CoG to rear axle (m)
	L_b 	= 1.4987		# from CoG to rear axle (according to Jongsang)
	C_r 	= 13.4851e4*2 
	C_f 	= 7.6419e4*2
	m 		= 2303.1 
	Iz 		= 5520.1


	# predicted (s,v); "init" for initial solve
    s_pred_init   = 1.0*collect(0:dt:(N)*dt)		# predicted s(0), s(1), ..., s(N)
	vx_pred_init = ones(N+1,1)						# predicted v(0), v(1), ..., v(N)
	k_coeff_init = [0.0, 0.0, 0.0, 0.0]		# K(s) = a0 + a1 * s + a2 * s^2 + a3 * s^3


	# system dynamics A, B, c defined later on (they depend on reference)
	A_init = zeros(nx, nx, N)
	B_init = zeros(nx, nu, N)
	g_init = zeros(nx, N)
	for i = 1:N
		# A_init[:,:,i] = [	1 		dt*vx_pred_init[i] 		dt 											0 
		# 					0		1						0											dt
		# 					0 		0						1-dt*(C_f+C_r)/(m*vx_pred_init[i])			dt*((L_b*C_r-L_a*C_f)/(m*vx_pred_init[i])-vx_pred_init[i])
		# 					0		0						dt*(-L_a*C_f+L_b*C_r)/(Iz*vx_pred_init[i]) 	1+dt*(-L_a^2*C_f-L_b^2*C_r)/(Iz*vx_pred_init[i])			]
		
		A_init[:,:,i] = expm(dt*[	0 		vx_pred_init[i] 		1 											0 
									0		0						0											1
									0 		0						-(C_f+C_r)/(m*vx_pred_init[i])				(L_b*C_r-L_a*C_f)/(m*vx_pred_init[i])-vx_pred_init[i]
									0		0						(-L_a*C_f+L_b*C_r)/(Iz*vx_pred_init[i]) 	(-L_a^2*C_f-L_b^2*C_r)/(Iz*vx_pred_init[i])			])

		B_init[:,:,i] = [	0
							0
							dt*C_f/m
							dt*L_a*C_f/Iz 	]

		g_init[:,i] = [ 	0	# column vector
							-dt*vx_pred_init[i]*(k_coeff_init[1]*s_pred_init[i]^3 + k_coeff_init[2]*s_pred_init[i]^2 + k_coeff_init[3]*s_pred_init[i] + k_coeff_init[4]) 	
							0
							0	]
	end

	# define cost functions
    C_ey 	= 5.0				# lateral deviation
    C_epsi 	= 0.0			# heading deviation
    C_vy   	= 0.0			# lateral velocity
    C_wz 	= 0.0			# yaw rate 


	C_df	 = 0.0	# 150			# tire angle input
	C_ddf	 = 1000.0	# 3e4			# derivative of tire angle input

	Q = diagm([C_ey ; C_epsi ; C_vy ; C_wz])	# state cost
	R = C_df 					# input cost
	Rdelta = C_ddf				# deltaU cost


	# define (box) constraints
	largeNumber = 1e2;		# use this number for variables that are not upper/lower bounded
	ey_min = -largeNumber
	ey_max = largeNumber
	epsi_min = -largeNumber
	epsi_max = largeNumber
	vy_min	= -largeNumber  	# ???
	vy_max 	= largeNumber		# ???
	wz_min	= -largeNumber 		# ???
	wz_max	= largeNumber 		# ???

	df_max = 0.5	# steering
	ddf_max = 0.5	# change in steering

	# collect constraint
	x_lb = [	ey_min
				epsi_min 
				vy_min
				wz_min		]
	x_ub = [	ey_max
				epsi_max 	
				vy_max 	
				wz_max		]

	u_lb = -df_max
	u_ub = df_max

	dU_lb = -ddf_max*dt
	dU_ub = ddf_max*dt


	# build references (which are updated)
	# get init x_ref for one initial solve (to speed up Julia)
	ey_ref_init = zeros(N) 		# regulate ey to zero
	epsi_ref_init = zeros(N)	# regulate epsi to zero (no weight)
	vy_ref_init = zeros(N)
	wz_ref_init = zeros(N)

	x_ref_init = zeros(N*nx)
	for i = 1 : N
		x_ref_init[(i-1)*nx+1] = ey_ref_init[i]		# set x_ref
		x_ref_init[(i-1)*nx+2] = epsi_ref_init[i]		# set v_ref
		x_ref_init[(i-1)*nx+3] = vy_ref_init[i]		# set v_ref
		x_ref_init[(i-1)*nx+4] = wz_ref_init[i]		# set v_ref
	end	

	# input reference
	u_ref_init = zeros(N,1)	# if not used, set cost to zeros

	# get Initial state and input
	# should be done dynamically later on
	ey0_init = 0
	epsi0_init = 0
	vy0_init = 0
	wz0_init = 0

	x0_init = [ey0_init ; epsi0_init ; vy0_init ; wz0_init]
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
	# println("objv = $(objv)
	

	# this function is called iteratively
	# may want to add epsi_ref in the future (epsi in a curve is generally non-zero)
	function solve_gurobi(ey_0::Float64, epsi_0::Float64, vy_0::Float64, wz_0::Float64, u_0::Float64, s_pred::Array{Float64,1}, vx_pred::Array{Float64,1}, k_coeffs::Array{Float64,1})
		# Quick check on reference velocity to prevent singularity issue in slip angle.
		vx_pred_denom = vx_pred
		for i = 1:length(vx_pred_denom)
			if vx_pred[i] < 1.0
				vx_pred_denom[i] = 1.0	
			end
		end	

		tic()

		# build problem
		x0 = [ey_0 ; epsi_0 ; vy_0 ; wz_0]
		u0 = u_0 				# it's really u_{-1}
		x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

		# system dynamics A, B, c defined later on (they depend on reference)
		A_updated = zeros(nx, nx, N)
		B_updated = zeros(nx, nu, N)
		g_updated = zeros(nx, N)
		for i = 1 : N
			# A_updated[:,:,i] = [	1 		dt*vx_pred[i] 			dt 											0 
			# 						0		1						0											dt
			# 						0 		0						1-dt*(C_f+C_r)/(m*vx_pred_denom[i])			dt*((L_b*C_r-L_a*C_f)/(m*vx_pred_denom[i])-vx_pred[i])
			# 						0		0						dt*(-L_a*C_f+L_b*C_r)/(Iz*vx_pred_denom[i]) 		1+dt*(-L_a^2*C_f-L_b^2*C_r)/(Iz*vx_pred_denom[i])			]

			A_updated[:,:,i] = expm(  dt*[0 		vx_pred[i] 				1 											0 
										0		0						0											1
										0 		0						-(C_f+C_r)/(m*vx_pred_denom[i])				(L_b*C_r-L_a*C_f)/(m*vx_pred_denom[i])-vx_pred[i]
										0		0						(-L_a*C_f+L_b*C_r)/(Iz*vx_pred_denom[i]) 	(-L_a^2*C_f-L_b^2*C_r)/(Iz*vx_pred_denom[i])		]  )


			B_updated[:,:,i] = [	0
									0
									dt*C_f/m
									dt*L_a*C_f/Iz 	]

			g_updated[:,i] = [ 		0	# column vector
							 		-dt*vx_pred[i]*(k_coeffs[1]*s_pred[i]^3 + k_coeffs[2]*s_pred[i]^2 + k_coeffs[3]*s_pred[i] + k_coeffs[4]) 	
									0
									0	]
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

		# println("====== ey0, epsi0, vy0, wz0 in Gurobi LAT DYN:  $(x0') ====== ")
		# println("u0 LAT in Gurobi: $(u_0)")

		# println("s_pred in Gurobi (incl s0): $(s_pred)")
		# println("vx_pred in Gurobi (incl v0): $(vx_pred)")
		# println("k_coeffs in Gurobi: $(k_coeffs)")

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


		# structure of z = [ (ddf,ey,epsi,vy, wz, df) ; (ddf, ey, epsi, vy, wz, df) ; ... ]
		ddf_pred_gurobi = optimizer_gurobi[1:n_uxu:end]
		df_pred_gurobi = optimizer_gurobi[6:n_uxu:end]
		ey_pred_gurobi = [ ey_0 ; optimizer_gurobi[2:n_uxu:end] ] 	# include current s 
		epsi_pred_gurobi = [ epsi_0 ; optimizer_gurobi[3:n_uxu:end] ]		# include current v 
		vy_pred_gurobi = [ vy_0 ; optimizer_gurobi[4:n_uxu:end] ]		# include current v 
		wz_pred_gurobi = [ wz_0 ; optimizer_gurobi[5:n_uxu:end] ]		# include current v 

		
		# println("ddf_pred_gurobi: $(ddf_pred_gurobi)")
		# println("df_pred_gurobi: $(df_pred_gurobi)")
		# println("ey_pred_gurobi (incl ey0): $(ey_pred_gurobi)")
		# println("epsi_pred_gurobi (incl epsi0): $(epsi_pred_gurobi)")
		# println("vy_pred_gurobi (incl vy0): $(vy_pred_gurobi)")
		# println("wz_pred_gurobi (incl wz0): $(wz_pred_gurobi)")
		# println(" ")


				# @printf "%.20f \n"  k_coeffs[1]
		# @printf "%.20f \n"  k_coeffs[2]
		# @printf "%.20f \n"  k_coeffs[3]
		# @printf "%.20f \n"  k_coeffs[4]

		df_opt = optimizer_gurobi[6]

	   	return df_opt, df_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, vy_pred_gurobi, wz_pred_gurobi, solvTimeGurobi1, status

	end  	# end of solve_gurobi()

end
