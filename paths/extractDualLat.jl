# code to test and extract dual multipliers from primal solution Lateral 
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
 

############## load all data ##############
latData = matread("CPG_day1_test1_trainingData.mat")   				# bad


inputParam_lat = latData["inputParam_lat"]   #
# outputParamDf_lat = latData["outputParamDf_lat"]
# outputParamDdf_lat = latData["outputParamDdf_lat"]

# parse initial data
ey_curr_all = inputParam_lat[:,1]
epsi_curr_all = inputParam_lat[:,2]
df_prev_all = inputParam_lat[:,3]
v_pred_all = inputParam_lat[:,4:4+N-1]

### Param Merge Here
c_pred_all = inputParam_lat[:,4+N:end]
vc_pred_all = v_pred_all .* c_pred_all
inputParam_latNew = inputParam_lat
inputParam_latNew[:,4+N:end] = vc_pred_all 		# NN trains with v*c

# vc_pred_all = inputParam_lat[:,4+N:end]
#########################################

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
# F_tilde = [eye(nx+nu) ; -eye(nx+nu)]
# f_tilde = [x_tilde_ub ; -x_tilde_lb]
# nf = length(f_tilde);
nf = 0
# Concatenate appended state (tilde) constraints
# F_tilde_vec = kron(eye(N), F_tilde)
# f_tilde_vec = repmat(f_tilde,N)   

######################## ITERATE OVER saved parameters ################
# build problem
# num_DataPoints = 10000								# Training data count 
num_DataPoints = size(inputParam_lat,1)

solv_time_all = zeros(num_DataPoints)

ddf_res_all = zeros(num_DataPoints)			# compares solution of computed problem with training (stored) data

dual_gap 	= zeros(num_DataPoints)
Reldual_gap = zeros(num_DataPoints)

outputParamDdf_lat  = zeros(num_DataPoints, N*(nu))
outputParamDual_lat = zeros(num_DataPoints, N*(nf+ng))
optVal_lat 			= zeros(num_DataPoints)

# counts number of errors when solving the optimization problems
primStatusError = 0
dualStatusError = 0

iii = 1

while iii <= num_DataPoints

	if mod(iii,100) == 0
		println("step: $(iii)")
	end
	 	
	###### stored data #####
	ey_0 = ey_curr_all[iii]
	epsi_0 = epsi_curr_all[iii]
	u_0 = df_prev_all[iii]
	v_pred = v_pred_all[iii,:]
	vc_pred = vc_pred_all[iii,:]
 	# load data to check if solution identical

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
						-dt*vc_pred[i] ] #param merge
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

	# need to build A_tilde_vec, B_tilde_vec, E_tilde_vec
	A_tilde_vec = zeros(N*(nx+nu), (nx+nu))
	A_tmp = eye(nx+nu)  	# tmp variable used to store the ``powers of A_tilde"
	for ii = 1 : N
		A_tmp = A_tilde_updated[:,:,ii]*A_tmp
	    A_tilde_vec[1+(ii-1)*(nx+nu):ii*(nx+nu),:] = A_tmp 	#A_tilde^ii
	end

	# new CORRECT construction method
	B_tilde_vec2 = zeros(N*(nx+nu), nu*N)
	B_tilde_vec2[1:(nx+nu),1:nu] = B_tilde_updated[:,:,1] 	# for stage 1: x1 = ... + B0 * u0  + ...
	for ii = 2 : N  # for stages x2, ...
		B_tilde_vec2[(ii-1)*(nx+nu)+1 : ii*(nx+nu) , :] = A_tilde_updated[:,:,ii] * B_tilde_vec2[(ii-2)*(nx+nu)+1 : (ii-1)*(nx+nu) , :]
		B_tilde_vec2[(ii-1)*(nx+nu)+1 : ii*(nx+nu) , (ii-1)*nu+1:ii*nu] = B_tilde_updated[:,:,ii]
	end

	nw=nx+nu
	# new approach: CORRECT
	E_tilde_vec2 = zeros(N*(nx+nu), nw*N)
	E_tilde_vec2[1:(nx+nu) , 1:nw] = eye(nw) 	# for x1
	for ii = 2 : N 		# for x2, x3, ...
		E_tilde_vec2[(ii-1)*nw+1 : ii*nw  , :  ] = 	A_tilde_updated[:,:,ii] * E_tilde_vec2[(ii-2)*nw+1 : (ii-1)*nw  , :  ]
		E_tilde_vec2[(ii-1)*nw+1 : ii*nw , (ii-1)*nw+1 : ii*nw ] = eye(nw)
	end

	E_tilde_vec = E_tilde_vec2
	B_tilde_vec = B_tilde_vec2

	g_tilde_vec = zeros(N*(nx+nu))
	for ii = 1 : N
		g_tilde_vec[1+(ii-1)*(nx+nu) : ii*(nx+nu)] = g_tilde_updated[:,ii]
	end

	x_tilde_ref = x_tilde_ref_init
	
	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, x_tilde_vec[1:N*(nx+nu)])  	# decision variable; contains everything
	@variable(mdl, u_tilde_vec[1:N*nu] )
	@objective(mdl, Min, (x_tilde_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_vec-x_tilde_ref) + u_tilde_vec'*R_tilde_vec*u_tilde_vec)
	constr_eq = @constraint(mdl, x_tilde_vec .== A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_vec + E_tilde_vec*g_tilde_vec)
	# constr_Fx = @constraint(mdl, F_tilde_vec*x_tilde_vec .<= f_tilde_vec)
	constr_Fu = @constraint(mdl, Fu_tilde_vec*u_tilde_vec .<= fu_tilde_vec)

	status = solve(mdl)
	if !(status == :Optimal)
		println(status)
		primStatusError = primStatusError+1
		@goto label1
	end


################################################
	#### extract dual variables ####
	Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
     
    c_dual = (2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*B_tilde_vec +
    	      - 2*x_tilde_ref'*Q_tilde_vec*B_tilde_vec)'
     
    const_dual = x_tilde_0'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0 + 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  + g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  - 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref - 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref +
                  + x_tilde_ref'*Q_tilde_vec*x_tilde_ref
        
    # C_dual = [F_tilde_vec*B_tilde_vec; Fu_tilde_vec ]	# Adding state constraints 
    # d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0 - F_tilde_vec*E_tilde_vec*g_tilde_vec;  fu_tilde_vec]
    C_dual = Fu_tilde_vec	# Adding state constraints 
    d_dual = fu_tilde_vec
    Qdual_tmp = C_dual*(Q_dual\(C_dual'))
    Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))
    
	# Solve the dual problem online to match cost 
    mdlD = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdlD, L_test[1:N*(nf+ng)])  	# decision variable; contains everything
	@objective(mdlD, Max, -1/2 * L_test'*Qdual_tmp*L_test - (C_dual*(Q_dual\c_dual)+d_dual)'*L_test - 1/2*c_dual'*(Q_dual\c_dual) + const_dual)
	@constraint(mdlD, -L_test .<= 0)

	statusD = solve(mdlD)
 	if !(statusD == :Optimal)
		println("****** PROBLEM IN DUAL ******")
		@goto label1 
	end


	############### collect data
	ddf_pred_opt = getvalue(u_tilde_vec)
	outputParamDdf_lat[iii,:] = ddf_pred_opt

	L_test_opt = getvalue(L_test)
	outputParamDual_lat[iii,:] = L_test_opt

	dual_gap[iii] = (getobjectivevalue(mdl) - getobjectivevalue(mdlD))
	Reldual_gap[iii] = dual_gap[iii] / getobjectivevalue(mdl)
	optVal_lat[iii] = getobjectivevalue(mdl)

 	###########################################################	

	# #### compare solution ####


	@label label1
	iii = iii + 1 

end

println("===========================================")

println(" ")


println("max dual_gap:  $(maximum(dual_gap))")
println("min dual_gap:  $(minimum(dual_gap))")
println("avg dual_gap:  $(mean(dual_gap))")

println(" ")

println("max Rel dual_gap:  $(maximum(Reldual_gap))")
println("min Rel dual_gap:  $(minimum(Reldual_gap))")
println("avg Rel dual_gap:  $(mean(Reldual_gap))")

matwrite("CPG_day1_test1_latTrainingDataUnConstr.mat", Dict(
	"inputParam_lat" => inputParam_lat,
	"outputParamDdf_lat" => outputParamDdf_lat,
	"outputParamDual_lat" => outputParamDual_lat,
	"optVal_lat" => optVal_lat
))
println("---- done extracting and saving dual for LAT control ----")
