# code to test and extract dual multipliers from primal solution
# GXZ + MB

using MAT
using Gurobi
using JuMP



#### problem paramters for LONGITUDINAL Control 
# Global variable LOAD_PATH contains the directories Julia searches for modules when calling require. It can be extended using push!:
push!(LOAD_PATH, "../scripts/mpc_utils") 	
import GPSKinMPCPathFollowerFrenetLinLongGurobi
const kmpcLinLong = GPSKinMPCPathFollowerFrenetLinLongGurobi  # short-hand-notation


# Load as Many parameters as possible from MPC file to avoid parameter mis-match
N 		= kmpcLinLong.N
dt 		= kmpcLinLong.dt
nx 		= kmpcLinLong.nx				# dimension of x = (ey,epsi)
nu 		= kmpcLinLong.nu				# number of inputs u = df
L_a 	= kmpcLinLong.L_a		# from CoG to front axle (according to Jongsang)
L_b 	= kmpcLinLong.L_b		# from CoG to rear axle (according to Jongsang)

############## load all data ##############
longData = matread("NN_test_trainingData.mat")

inputParam_long = longData["inputParam_long"]   # np.hstack((s_curr.T, v_curr.T ,a_prev.T, s_ref, v_ref ))
outputParamAcc_long = longData["outputParamAcc_long"]
outputParamDacc_long = longData["outputParamDacc_long"]

# parse initial data
s_curr_all = inputParam_long[:,1]
v_curr_all = inputParam_long[:,2]
a_prev_all = inputParam_long[:,3]
s_ref_all = inputParam_long[:,4:4+N-1]
v_ref_all = inputParam_long[:,12:end]

######### Define parameters that don't change ############
# define System matrices (all time-invariant)
# in principle, these should all be read from the long-file (in case stuff change)
# A = kmpcLinLong.A
# # A = [	1 	dt 		# can be made more exact using matrix exponential
# # 		0	1 	]
# B = kmpcLinLong.B
# # B = [ 	0
# 		# dt 		]
# g = kmpcLinLong.g
# # g = [	0
# 		# 0	]
# # define cost functions
# C_s = kmpcLinLong.C_s			# track progress
# C_v = kmpcLinLong.C_v			# ref velocity tracking weight			
# C_acc = kmpcLinLong.C_acc
# C_dacc = kmpcLinLong.C_dacc;		# 20 too high; 10 OK for med speed; 10 a big jerky for high speed; 13 too high

# Q = kmpcLinLong.Q
# R = kmpcLinLong.R
# Rdelta = kmpcLinLong.Rdelta

# # define (box) constraints
# largeNumber = kmpcLinLong.largeNumber;		# use this number for variables that are not upper/lower bounded
# v_min = kmpcLinLong.v_min				# vel bounds (m/s)
# v_max = kmpcLinLong.v_max	
# a_max = kmpcLinLong.a_max				# acceleration and deceleration bound, m/s^2
# a_dmax = kmpcLinLong.a_dmax			# jerk bound, m/s^3

# x_lb = kmpcLinLong.x_lb
# x_ub = kmpcLinLong.x_ub

# u_lb = kmpcLinLong.u_lb
# u_ub = kmpcLinLong.u_ub

# dU_lb = kmpcLinLong.dU_lb 	# double check if *dt is needed (or not)
# dU_ub = kmpcLinLong.dU_ub


# input reference
u_ref_init = kmpcLinLong.u_ref_init	# if not used, set cost to zeros


# ================== Transformation 1 ======================
# augment state and redefine system dynamics (A,B,g) and constraints
# x_tilde_k := (x_k , u_{k-1})
# u_tilde_k := (u_k - u_{k-1})

A_tilde = kmpcLinLong.A_tilde

# B_tilde = kmpcLinLong.B_tilde

g_tilde = kmpcLinLong.g_tilde

# x_tilde_lb = kmpcLinLong.x_tilde_lb
# x_tilde_ub = kmpcLinLong.x_tilde_ub
# u_tilde_lb = kmpcLinLong.u_tilde_lb
# u_tilde_ub = kmpcLinLong.u_tilde_ub

# Q_tilde = kmpcLinLong.Q_tilde
# Q_tilde = [	Q 			zeros(nx,nu) 		# may also use cat(?)
			# zeros(nu,nx)	R 			]	# actually not needed

# R_tilde = kmpcLinLong.R_tilde
# R_tilde = Rdelta


# u_tilde_ref_init = kmpcLinLong.u_tilde_ref_init 	# goal is to minimize uTilde = (acc_k - acc_{k-1})

# # ================== Transformation 2 ======================
# # bring into GUROBI format
# # minimize_z    z' * H * z + f' * z
# #	s.t.		A_eq * z = b_eq
# #				A * z <= b
# #				z_lb <= z <= z_ub

# z := (u_tilde_0, x_tilde_1 , u_tilde_1 x_tilde_2 , ... u_tilde_{N-1}, x_tilde_N , )
n_uxu = kmpcLinLong.n_uxu	# size of one block of (u_tilde, x_tilde) = (deltaU, x, u)

# Build cost function
# cost for (u_tilde, x_tilde) = (deltaU , S, V, U)
# H_block = kmpcLinLong.H_block
H_gurobi = kmpcLinLong.H_gurobi


# build box constraints lb_gurobi <= z <= ub_gurobi
# recall: z = (u_tilde, x_tilde, ....)
lb_gurobi = kmpcLinLong.lb_gurobi		# (deltaU, X, U)
ub_gurobi = kmpcLinLong.ub_gurobi		# (deltaU, X, U)


# build equality matrix (most MALAKA task ever)
nu_tilde = kmpcLinLong.nu_tilde
nx_tilde = kmpcLinLong.nx_tilde
Aeq_gurobi = kmpcLinLong.Aeq_gurobi
# n_uxu = nu_tilde + nx_tilde
# Aeq_gurobi = zeros(N*nx_tilde , N*(nx_tilde+nu_tilde))
# Aeq_gurobi[1:nx_tilde, 1:(nx_tilde+nu_tilde)] = [-B_tilde eye(nx_tilde)] 	# fill out first row associated with x_tilde_1
# for i = 2 : N  	# fill out rows associated to x_tilde_2, ... , x_tilde_N
	# Aeq_gurobi[ (i-1)*nx_tilde+1 : i*nx_tilde  , (i-2)*(nu_tilde+nx_tilde)+(nu_tilde)+1 : (i-2)*(nu_tilde+nx_tilde)+nu_tilde+(nx_tilde+nu_tilde+nx_tilde)    ] = [-A_tilde -B_tilde eye(nx_tilde)]
# end


######################## ITERATE OVER saved parameters ################
# build problem
num_DataPoints = length(s_curr_all)
solv_time_all = zeros(num_DataPoints)
a_res_all = zeros(num_DataPoints)
dA_res_all = zeros(num_DataPoints)

outputParamDualEQ_long = zeros(num_DataPoints, N*(nx+nu))
outputParamDualLB_long = zeros(num_DataPoints, N*n_uxu)
outputParamDualUB_long = zeros(num_DataPoints, N*n_uxu)

dual_eq = []
dual_ub = []
dual_lb = []

for ii = 1 : num_DataPoints	
	# extract appropriate parameters
	s_0 = s_curr_all[ii]
	v_0 = v_curr_all[ii]
	u_0 = a_prev_all[ii]
	s_ref = s_ref_all[ii,:]
	v_ref = v_ref_all[ii,:]
	acc_stored = outputParamAcc_long[ii,:]
	dAcc_stored = outputParamDacc_long[ii,:]


	x0 = [s_0 ; v_0]
	u0 = u_0 				# it's really u_{-1}
	x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

	# update RHS of linear equality constraint
	beq_gurobi_updated = repmat(g_tilde,N,1);
	beq_gurobi_updated[1:nx_tilde] = beq_gurobi_updated[1:nx_tilde] + A_tilde*x_tilde_0 	# PARAMETER: depends on x0

	# update reference trajectories
	x_ref = zeros(N*nx,1)
	for i = 1 : N
		x_ref[(i-1)*nx+1] = s_ref[i]		# set x_ref, s_ref/v_ref is of dim N+1; index of s_ref changed
		x_ref[(i-1)*nx+2] = v_ref[i]		# set v_ref
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

	## Adding the dual inequality constraint matrices in Gz <= g format
	G_gurobi = [eye(N*n_uxu); -eye(N*n_uxu)]
	g_gurobi = [ub_gurobi; -lb_gurobi]

	################################################################## 
	# # OLD: formulate optimization problem
	# GurobiEnv = Gurobi.Env()	# really necessary?
	# setparam!(GurobiEnv, "Presolve", -1)		# -1: automatic; no big influence on solution time
	# setparam!(GurobiEnv, "LogToConsole", 0)		# set presolve to 0
	# # setparam!(GurobiEnv, "TimeLimit",0.025)		# for 20Hz = 50ms
	# # Formulate Optimization Problem
	#  	GurobiModel = gurobi_model(GurobiEnv;
	# 		name = "qp_01",
	# 		H = 2*H_gurobi,
	# 		f = f_gurobi_updated,	# need to make it "flat"
	# 		Aeq = Aeq_gurobi,
	# 		beq = squeeze(beq_gurobi_updated,2), # need to make it "flat"
	# 		lb = squeeze(lb_gurobi,2), # need to make it "flat"
	# 		ub = squeeze(ub_gurobi,2)	) # need to make it "flat"
	# optimize(GurobiModel)		 		# solve optimization problem
	# 	solvTimeGurobi1 = toq()
	# optimizer_gurobi = get_solution(GurobiModel)
	# status = get_status(GurobiModel)

	# Solve optimization problem
	# ================== recall Transformation 2 ======================
	# bring into GUROBI format
	# minimize_z    z' * H * z + f' * z
	#	s.t.		A_eq * z = b_eq
	#				A * z <= b
	#				z_lb <= z <= z_ub
	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, z[1:N*n_uxu])  	# decision variable; contains everything
	@objective(mdl, Min, z'*H_gurobi*z + f_gurobi_updated'*z)
	constr_eq = @constraint(mdl, Aeq_gurobi*z .== squeeze(beq_gurobi_updated,2))
	constr_ub = @constraint(mdl, z .<= squeeze(ub_gurobi,2))
	constr_lb = @constraint(mdl, -z .<= -squeeze(lb_gurobi,2))

	tic()
	status = solve(mdl)
	solv_time_all[ii] = toq()

	# extract solution
	z_opt = getvalue(z)
	# structure of z = [ (dAcc,s,v,Acc) ; (dAcc, s, v, Acc) ; ... ]
	dA_pred_opt = z_opt[1:n_uxu:end]
	a_pred_opt = z_opt[4:n_uxu:end]
	s_pred_opt = z_opt[2:n_uxu:end]  	# does not include s0
	v_pred_opt = z_opt[3:n_uxu:end] 		# does not include v0 

	#### compare solution ####
	a_res_all[ii] = norm(a_pred_opt - acc_stored)
	dA_res_all[ii] = norm(dA_pred_opt - dAcc_stored)

	#### extract dual variables ####
	dual_eq = getdual(constr_eq)
	dual_ub = getdual(constr_ub)
	dual_lb = getdual(constr_lb)

	dual_ineq = [dual_ub; dual_lb]


	outputParamDualEQ_long[ii,:] = dual_eq
	outputParamDualLB_long[ii,:] = dual_ub
	outputParamDualUB_long[ii,:] = dual_ub


	## Check the cost of the dual optimization problem and check if it matches 
	H_gurobi_dual = 2*H_gurobi
	z_opt_uc = -inv(H_gurobi_dual)*(f_gurobi_updated + G_gurobi'*dual_ineq + Aeq_gurobi'*dual_eq)
	opt_val_dual = 0.5*z_opt_uc'*H_gurobi_dual*z_opt_uc + (f_gurobi_updated' + dual_ineq'*G_gurobi + dual_eq'*Aeq_gurobi)*z_opt_uc - dual_ineq'*g_gurobi- dual_eq'*beq_gurobi_updated

	#########################################################################

end

println("max a-residual:  $(maximum(a_res_all))")
println("max dA-residual:  $(maximum(dA_res_all))")
println("max solv-time (excl modelling time):  $(maximum(solv_time_all[3:end]))")
println("avg solv-time (excl modelling time):  $(mean(solv_time_all[3:end]))")

matwrite("NN_test_trainingData_PrimalDual.mat", Dict(
	"inputParam_long" => inputParam_long,
	"outputParamAcc_long" => outputParamAcc_long,
	"outputParamDacc_long" => outputParamDacc_long,
	"outputParamDualEQ_long" => outputParamDualEQ_long,
	"outputParamDualLB_long" => outputParamDualLB_long,
	"outputParamDualUB_long" => outputParamDualUB_long
))


println("---- done extracting and saving dual for LONG control ----")
