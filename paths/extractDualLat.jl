# code to test and extract dual multipliers from primal solution
# GXZ + MB

using MAT
using Gurobi
using JuMP



#### problem paramters for LONGITUDINAL Control 
# Global variable LOAD_PATH contains the directories Julia searches for modules when calling require. It can be extended using push!:
push!(LOAD_PATH, "../scripts/mpc_utils") 	
import GPSKinMPCPathFollowerFrenetLinLatGurobi
const kmpcLinLat = GPSKinMPCPathFollowerFrenetLinLatGurobi  # short-hand-notation


# Load as Many parameters as possible from MPC file to avoid parameter mis-match
N 		= kmpcLinLat.N
dt 		= kmpcLinLat.dt
nx 		= kmpcLinLat.nx				# dimension of x = (ey,epsi)
nu 		= kmpcLinLat.nu				# number of inputs u = df
L_a 	= kmpcLinLat.L_a		# from CoG to front axle (according to Jongsang)
L_b 	= kmpcLinLat.L_b		# from CoG to rear axle (according to Jongsang)

############## load all data ##############
latData = matread("NN_test_trainingData.mat")

# inputParam_lat = np.hstack((ey_curr.T, epsi_curr.T ,df_prev.T, v_pred, c_pred))
inputParam_lat = latData["inputParam_lat"]   #
outputParamAcc_long = latData["outputParamAcc_long"]
outputParamDacc_long = latData["outputParamDacc_long"]

# parse initial data
s_curr_all = inputParam_long[:,1]
v_curr_all = inputParam_long[:,2]
a_prev_all = inputParam_long[:,3]
s_ref_all = inputParam_long[:,4:4+N-1]
v_ref_all = inputParam_long[:,12:end]


u_ref_init = kmpcLinLat.u_ref_init	# if not used, set cost to zeros


# ================== Transformation 1 ======================
# augment state and redefine system dynamics (A,B,g) and constraints
# x_tilde_k := (x_k , u_{k-1})
# u_tilde_k := (u_k - u_{k-1})

A_tilde = kmpcLinLat.A_tilde

g_tilde = kmpcLinLat.g_tilde

# # ================== Transformation 2 ======================
# # bring into GUROBI format
# # minimize_z    z' * H * z + f' * z
# #	s.t.		A_eq * z = b_eq
# #				A * z <= b
# #				z_lb <= z <= z_ub

# z := (u_tilde_0, x_tilde_1 , u_tilde_1 x_tilde_2 , ... u_tilde_{N-1}, x_tilde_N , )
n_uxu = kmpcLinLat.n_uxu	# size of one block of (u_tilde, x_tilde) = (deltaU, x, u)

# Build cost function
# cost for (u_tilde, x_tilde) = (deltaU , S, V, U)
H_gurobi = kmpcLinLat.H_gurobi


# build box constraints lb_gurobi <= z <= ub_gurobi
# recall: z = (u_tilde, x_tilde, ....)
lb_gurobi = kmpcLinLat.lb_gurobi		# (deltaU, X, U)
ub_gurobi = kmpcLinLat.ub_gurobi		# (deltaU, X, U)


# build equality matrix (most MALAKA task ever)
nu_tilde = kmpcLinLat.nu_tilde
nx_tilde = kmpcLinLat.nx_tilde
Aeq_gurobi = kmpcLinLat.Aeq_gurobi


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

for ii = 1 : 1	
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

	outputParamDualEQ_long[ii,:] = dual_eq
	outputParamDualLB_long[ii,:] = dual_ub
	outputParamDualUB_long[ii,:] = dual_ub

end

println("max a-residual:  $(maximum(a_res_all))")
println("max dA-residual:  $(maximum(dA_res_all))")
println("max solv-time (excl modelling time):  $(maximum(solv_time_all[3:end]))")
println("avg solv-time (excl modelling time):  $(mean(solv_time_all[3:end]))")

matwrite("NN_test_trainingDataLat_PrimalDual.mat", Dict(
	"inputParam_long" => inputParam_long,
	"outputParamAcc_long" => outputParamAcc_long,
	"outputParamDacc_long" => outputParamDacc_long,
	"outputParamDualEQ_long" => outputParamDualEQ_long,
	"outputParamDualLB_long" => outputParamDualLB_long,
	"outputParamDualUB_long" => outputParamDualUB_long
))


println("---- done extracting and saving dual for LONG control ----")
