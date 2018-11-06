# code to test and extract dual multipliers from primal solution Lateral 
# Data randomly generated and transformation 2 is used 
# GXZ + MB

using MAT
using Gurobi
using JuMP


#### problem paramters for LONGITUDINAL Control 
# Global variable LOAD_PATH contains the directories Julia searches for modules when calling require. It can be extended using push!:
push!(LOAD_PATH, "../scripts/mpc_utils") 	
import GPSKinMPCPathFollowerFrenetLinLatGurobi
import KinMPCParams
const kmpcLinLat = GPSKinMPCPathFollowerFrenetLinLatGurobi  # short-hand-notation


# Load as Many parameters as possible from MPC file to avoid parameter mis-match
N 		= KinMPCParams.N
dt 		= KinMPCParams.dt
nx 		= 2								# dimension of x = (ey,epsi)
nu 		= 1								# number of inputs u = df
L_a 	= KinMPCParams.L_a				# from CoG to front axle (according to Jongsang)
L_b 	= KinMPCParams.L_b				# from CoG to rear axle (according to Jongsang)


n_uxu 	= kmpcLinLat.n_uxu
H_gurobi = kmpcLinLat.H_gurobi
f_gurobi_init = kmpcLinLat.f_gurobi_init
ub_gurobi = kmpcLinLat.ub_gurobi
lb_gurobi = kmpcLinLat.lb_gurobi

## Load Ranges of params 
ey_lb = KinMPCParams.ey_lb
ey_ub = KinMPCParams.ey_ub
epsi_lb = KinMPCParams.epsi_lb
epsi_ub = KinMPCParams.epsi_ub
dfprev_lb = -KinMPCParams.dfprev_ub
dfprev_ub =  KinMPCParams.dfprev_lb

v_lb = KinMPCParams.vpred_lb 
v_ub = KinMPCParams.vpred_ub
curv_lb = KinMPCParams.curv_lb
curv_ub = KinMPCParams.curv_ub


### Load MPC data ###
x_tilde_lb = kmpcLinLat.x_tilde_lb
x_tilde_ub = kmpcLinLat.x_tilde_ub
u_tilde_lb = kmpcLinLat.u_tilde_lb
u_tilde_ub = kmpcLinLat.u_tilde_ub

Q_tilde = kmpcLinLat.Q_tilde
R_tilde = kmpcLinLat.R_tilde

nu_tilde = kmpcLinLat.nu_tilde
nx_tilde = kmpcLinLat.nx_tilde

Q_tilde_vec = kron(eye(N),Q_tilde)   # for x_tilde_vec
R_tilde_vec = kron(eye(N),R_tilde)	 # for u_tilde_vec

u_ref_init = kmpcLinLat.u_ref_init	# if not used, set cost to zeros
x_tilde_ref_init = kmpcLinLat.x_tilde_ref_init

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

######################## ITERATE OVER saved parameters ################
# build problem
num_DataPoints = 30000								# Training data count 
# num_DataPoints = size(inputParam_lat,1)

solv_time_all = zeros(num_DataPoints)
optVal_long = zeros(num_DataPoints)

inputParam_lat = 	  zeros(num_DataPoints,3+2*N)
outputParamDual_lat = zeros(num_DataPoints, N*(nf+ng))
outputParamDf_lat = zeros(num_DataPoints, N*(nu))
outputParamDdf_lat  = zeros(num_DataPoints, N*(nu))

status = []
statusD = []

# counts number of errors when solving the optimization problems
primStatusError = 0
dualStatusError = 0

ey_0 = []
epsi_0 = []
u_0 = []
v_pred = []
c_pred = []
df_stored = []
ddf_stored = []

dual_Fx = []
dual_Fu = []
L_test_opt = []

iii = 1

while iii <= num_DataPoints
	
	# Save only feasible points. 
	# extract appropriate parameters	
	# RANDOM data extraction
 	ey_0 = ey_lb + (ey_ub-ey_lb)*rand(1)				
 	epsi_0 = epsi_lb + (epsi_ub-epsi_lb)*rand(1) 
 	u_0 = dfprev_lb + (dfprev_ub-dfprev_lb)*rand(1) 		
	v_pred = v_lb + (v_ub-v_lb)*rand(1,N)						#  Along horizon 
	c_pred = curv_lb + (curv_ub-curv_lb)*rand(1,N)				#  Along horizon 
 	
 	# build problem (only the updated parts)
	x0 = [ey_0 ; epsi_0]
	u0 = u_0 				# it's really u_{-1}
	x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

	# system dynamics A, B, g
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

	
###########################

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
	# 	s.t.		A_eq * z = b_eq
	# 				A * z <= b
	# 				z_lb <= z <= z_ub

	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, z[1:N*n_uxu])  	# decision variable; contains everything
	@objective(mdl, Min, z'*H_gurobi*z + f_gurobi_init'*z)
	constr_eq = @constraint(mdl, Aeq_gurobi_updated*z .== beq_gurobi_updated)
	constr_ub = @constraint(mdl, z .<= ub_gurobi)
	constr_lb = @constraint(mdl, -z .<= -lb_gurobi)

	tic()
	status = solve(mdl)


	if !(status == :Optimal)
		println(status)
		primStatusError = primStatusError+1
		@goto label1
	end

	solv_time_all[iii] = toq()
	obj_primal2 = getobjectivevalue(mdl)

	# extract solution
	z_opt = getvalue(z)
		# structure of z = [ (ddf,ey,epsi,df) ; (ddf, ey, epsi, df) ; ... ]
	ddf_pred_opt2 = z_opt[1:n_uxu:end]
	df_pred_opt2 = z_opt[4:n_uxu:end]
	ey_pred_opt2 = z_opt[2:n_uxu:end]  	# does not include s0
	epsi_pred_opt2 = z_opt[3:n_uxu:end] 		# does not include v0 

	# store the primal solution too as output gonna change now 
	inputParam_lat[iii,:] = [ey_0 epsi_0 u_0 v_pred c_pred]
	outputParamDf_lat[iii,:]   = df_pred_opt2
	outputParamDdf_lat[iii,:]  = ddf_pred_opt2
 
	iii = iii + 1 

	@label label1
end

println("****************************")
println("primal status errors:  $(primStatusError)")
println("dual status errors:  $(dualStatusError)")


matwrite("NN_test_CPGDay2BacktoDay1Tune_RandDataLat30kTrafo2.mat", Dict(
	"inputParam_lat" => inputParam_lat,
	"outputParamDf_lat" => outputParamDf_lat,
	"outputParamDdf_lat" => outputParamDdf_lat,
	"outputParamDual_lat" => outputParamDual_lat
))
println("---- done extracting and saving solutions for LAT control ----")
