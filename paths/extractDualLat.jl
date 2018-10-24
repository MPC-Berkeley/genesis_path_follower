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
outputParamDf_lat = latData["outputParamDf_lat"]
outputParamDdf_lat = latData["outputParamDdf_lat"]

# parse initial data
ey_curr_all = inputParam_lat[:,1]
epsi_curr_all = inputParam_lat[:,2]
df_prev_all = inputParam_lat[:,3]
v_pred_all = inputParam_lat[:,4:4+N-1]
c_pred_all = inputParam_lat[:,12:end]


u_ref_init = kmpcLinLat.u_ref_init	# if not used, set cost to zeros


# ================== Transformation 1 ======================
# augment state and redefine system dynamics (A,B,g) and constraints
# x_tilde_k := (x_k , u_{k-1})
# u_tilde_k := (u_k - u_{k-1})

# A_tilde = kmpcLinLat.A_tilde

# g_tilde = kmpcLinLat.g_tilde

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
f_gurobi_init = kmpcLinLat.f_gurobi_init

# build box constraints lb_gurobi <= z <= ub_gurobi
# recall: z = (u_tilde, x_tilde, ....)
lb_gurobi = kmpcLinLat.lb_gurobi		# (deltaU, X, U)
ub_gurobi = kmpcLinLat.ub_gurobi		# (deltaU, X, U)


# build equality matrix (most MALAKA task ever)
nu_tilde = kmpcLinLat.nu_tilde
nx_tilde = kmpcLinLat.nx_tilde
# Aeq_gurobi = kmpcLinLat.Aeq_gurobi


######################## ITERATE OVER saved parameters ################
# build problem
num_DataPoints = length(ey_curr_all)
solv_time_all = zeros(num_DataPoints)
df_res_all = zeros(num_DataPoints)
ddf_res_all = zeros(num_DataPoints)

outputParamDualEQ_lat = zeros(num_DataPoints, N*(nx+nu))
outputParamDualLB_lat = zeros(num_DataPoints, N*n_uxu)
outputParamDualUB_lat = zeros(num_DataPoints, N*n_uxu)

dual_eq = []
dual_ub = []
dual_lb = []

for ii = 1 : num_DataPoints	
	# extract appropriate parameters
	ey_0 = ey_curr_all[ii]
	epsi_0 = epsi_curr_all[ii]
	u_0 = df_prev_all[ii]
	v_pred = v_pred_all[ii,:]
	c_pred = c_pred_all[ii,:]
	df_stored = outputParamDf_lat[ii,:]
	ddf_stored = outputParamDdf_lat[ii,:]

	# build problem (only the updated parts)
	x0 = [ey_0 ; epsi_0]
	u0 = u_0 				# it's really u_{-1}
	x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

	# system dynamics A, B, c defined later on (they depend on reference)
	A_updated = zeros(nx, nx, N)
	B_updated = zeros(nx, nu, N)
	g_updated = zeros(nx, N)
	for i = 1 : N
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



	# Solve optimization problem
	# ================== recall Transformation 2 ======================
	# bring into GUROBI format
	# minimize_z    z' * H * z + f' * z
	#	s.t.		A_eq * z = b_eq
	#				A * z <= b
	#				z_lb <= z <= z_ub
	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, z[1:N*n_uxu])  	# decision variable; contains everything
	@objective(mdl, Min, z'*H_gurobi*z + f_gurobi_init'*z)
	constr_eq = @constraint(mdl, Aeq_gurobi_updated*z .== beq_gurobi_updated)
	constr_ub = @constraint(mdl, z .<= ub_gurobi)
	constr_lb = @constraint(mdl, -z .<= -lb_gurobi)

	tic()
	status = solve(mdl)
	solv_time_all[ii] = toq()

	# extract solution
	z_opt = getvalue(z)
		# structure of z = [ (ddf,ey,epsi,df) ; (ddf, ey, epsi, df) ; ... ]
	ddf_pred_opt = z_opt[1:n_uxu:end]
	df_pred_opt = z_opt[4:n_uxu:end]
	ey_pred_opt = z_opt[2:n_uxu:end]  	# does not include s0
	epsi_pred_opt = z_opt[3:n_uxu:end] 		# does not include v0 

	#### compare solution ####
	df_res_all[ii] = norm(df_pred_opt - df_stored)
	ddf_res_all[ii] = norm(ddf_pred_opt - ddf_stored)

	#### extract dual variables ####
	dual_eq = getdual(constr_eq)
	dual_ub = getdual(constr_ub)
	dual_lb = getdual(constr_lb)

	outputParamDualEQ_lat[ii,:] = dual_eq
	outputParamDualLB_lat[ii,:] = dual_ub
	outputParamDualUB_lat[ii,:] = dual_ub

end

println("max df-residual:  $(maximum(df_res_all))")
println("max ddf-residual:  $(maximum(ddf_res_all))")
println("max solv-time (excl modelling time):  $(maximum(solv_time_all[2:end]))")
println("avg solv-time (excl modelling time):  $(mean(solv_time_all[2:end]))")

matwrite("NN_test_trainingDataLat_PrimalDual.mat", Dict(
	"inputParam_lat" => inputParam_lat,
	"outputParamDf_lat" => outputParamDf_lat,
	"outputParamDdf_lat" => outputParamDdf_lat,
	"outputParamDualEQ_lat" => outputParamDualEQ_lat,
	"outputParamDualLB_lat" => outputParamDualLB_lat,
	"outputParamDualUB_lat" => outputParamDualUB_lat
))



println("---- done extracting and saving dual for LAT control ----")
