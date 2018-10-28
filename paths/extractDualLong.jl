# code to test and extract dual multipliers from primal solution
# GXZ + MB

using MAT
using Gurobi
using JuMP


#### problem paramters for LONGITUDINAL Control 
# Global variable LOAD_PATH contains the directories Julia searches for modules when calling require. It can be extended using push!:
push!(LOAD_PATH, "../scripts/mpc_utils") 	
import GPSKinMPCPathFollowerFrenetLinLongGurobi
import KinMPCParams
const kmpcLinLong = GPSKinMPCPathFollowerFrenetLinLongGurobi  # short-hand-notation


# Load as Many parameters as possible from MPC file to avoid parameter mis-match
N 		= KinMPCParams.N
dt 		= KinMPCParams.dt
nx 		= 2								# dimension of x = (ey,epsi)
nu 		= 1								# number of inputs u = df
L_a 	= KinMPCParams.L_a				# from CoG to front axle (according to Jongsang)
L_b 	= KinMPCParams.L_b				# from CoG to rear axle (according to Jongsang)

############## load all data ##############
# longData = matread("NN_test_trainingData.mat")

# inputParam_long = longData["inputParam_long"]   # np.hstack((s_curr.T, v_curr.T ,a_prev.T, s_ref, v_ref ))
# outputParamAcc_long = longData["outputParamAcc_long"]
# outputParamDacc_long = longData["outputParamDacc_long"]

## Load Ranges of params 

 v_lb = KinMPCParams.v_min 
 v_ub = KinMPCParams.v_max
 aprev_lb = -KinMPCParams.a_max
 aprev_ub =  KinMPCParams.a_max
 
###### Merge random data with extra data from vehicle path following    
# inputParam_long =  [s_train_rand v_train_rand aprev_train_rand s_ref_train_rand v_ref_train_rand ]

# s_curr_all = inputParam_long[:,1]
# v_curr_all = inputParam_long[:,2]
# a_prev_all = inputParam_long[:,3]
# s_ref_all =  inputParam_long[:,4:4+N-1]
# v_ref_all =  inputParam_long[:,12:end]								# All data appended 

# input reference
u_ref_init = kmpcLinLong.u_ref_init									# if not used, set cost to zeros

# ================== Transformation 1 =======================
# augment state and redefine system dynamics (A,B,g) and constraints
# x_tilde_k := (x_k , u_{k-1})
# u_tilde_k := (u_k - u_{k-1})

A_tilde = kmpcLinLong.A_tilde
B_tilde = kmpcLinLong.B_tilde
g_tilde = kmpcLinLong.g_tilde

x_tilde_lb = kmpcLinLong.x_tilde_lb
x_tilde_ub = kmpcLinLong.x_tilde_ub
u_tilde_lb = kmpcLinLong.u_tilde_lb
u_tilde_ub = kmpcLinLong.u_tilde_ub

Q_tilde = kmpcLinLong.Q_tilde
R_tilde = kmpcLinLong.R_tilde


# build equality matrix (most MALAKA task ever)
nu_tilde = kmpcLinLong.nu_tilde
nx_tilde = kmpcLinLong.nx_tilde

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


u_ref_init = zeros(N,1)	# if not used, set cost to zeros

# build constraints
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
num_DataPoints = 1000
solv_time_all = zeros(num_DataPoints)
a_res_all = zeros(num_DataPoints)
dA_res_all = zeros(num_DataPoints)
dual_gap = zeros(num_DataPoints)
Reldual_gap = zeros(num_DataPoints)
optVal_long = zeros(num_DataPoints)

outputParamDual_long = zeros(num_DataPoints, N*(nf+ng))

dual_Fx = []
dual_Fu = []
L_test_opt = []

inputParam_long = zeros(num_DataPoints,3+2*N)
outputParamDacc_long = zeros(num_DataPoints, N*(nu))
outputParamAcc_long  = zeros(num_DataPoints, N*(nu))
s_ub_ref = zeros(1,N)
ii = 1

for refC = 1:N
	s_ub_ref[1,refC] = 3 + 2*(refC-1) 					# Increments of 2 along horizon 
end

while ii <= num_DataPoints
	
	# Save only feasible points. 
	# extract appropriate parameters	
 	s_0 = -1 + 2*rand(1)								# Normalized to 0 now apparently  
 	v_0 = v_lb + (v_ub-v_lb)*rand(1) 
 	u_0 = aprev_lb + (aprev_ub-aprev_lb)*rand(1) 		
	s_ref = rand(1)*s_ub_ref							# Vary along horizon 
 	v_ref = v_lb + (v_ub-v_lb)*rand(1,N)				
 
	# acc_stored = outputParamAcc_long[ii,:]			
	# dAcc_stored = outputParamDacc_long[ii,:]			


	x_ref = zeros(N*nx,1)
	for i = 1 : N
		x_ref[(i-1)*nx+1] = s_ref[i]		# set x_ref, s_ref/v_ref is of dim N+1; index of s_ref changed
		x_ref[(i-1)*nx+2] = v_ref[i]		# set v_ref
	end
	x_tilde_ref = zeros(N*(nx+nu))
	for i = 1 : N
		x_tilde_ref[(i-1)*(nx+nu)+1 : (i-1)*(nx+nu)+nx] = x_ref[(i-1)*nx+1 : (i-1)*nx+nx]
		x_tilde_ref[(i-1)*(nx+nu)+nx+1 : (i-1)*(nx+nu)+nx+nu] = u_ref_init[i]	# u_ref_init always 0, but no no weights
	end

	x0 = [s_0 ; v_0]
	u0 = u_0 				# it's really u_{-1}
	x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER

	# solve primal problem
	# use it to test consistency
	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, x_tilde_vec[1:N*(nx+nu)])  	# decision variable; contains everything
	@variable(mdl, u_tilde_vec[1:N*nu] )
	@objective(mdl, Min, (x_tilde_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_vec-x_tilde_ref) + u_tilde_vec'*R_tilde_vec*u_tilde_vec)
	constr_eq = @constraint(mdl, x_tilde_vec .== A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_vec + E_tilde_vec*g_tilde_vec)
	constr_Fx = @constraint(mdl, F_tilde_vec*x_tilde_vec .<= f_tilde_vec)
	constr_Fu = @constraint(mdl, Fu_tilde_vec*u_tilde_vec .<= fu_tilde_vec)

	tic()
	status = solve(mdl)
	obj_primal = getobjectivevalue(mdl)

 	if !(status == :Optimal)
 		@goto label1 
 	end

# 	#### compare solution with stored solutions ####
# 	# a_res_all[ii] = norm(a_pred_opt - acc_stored)
# 	# dA_res_all[ii] = norm(dA_pred_opt - dAcc_stored)

# 	#### extract dual variables ####
# 	### seems to be wrong 
# 	# dual_eq = getdual(constr_eq)
# 	# dual_Fx = getdual(constr_Fx)
# 	# dual_Fu = getdual(constr_Fu)
# 	# dual_ineq = [dual_Fx; dual_Fu]

	#### compute dual cost ####
	#### get dual variables ###
	Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
     
    c_dual = (2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*B_tilde_vec +
    	      - 2*x_tilde_ref'*Q_tilde_vec*B_tilde_vec)'
     
    const_dual = x_tilde_0'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0 + 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  + g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  - 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref - 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref +
                  + x_tilde_ref'*Q_tilde_vec*x_tilde_ref
        
    C_dual = [F_tilde_vec*B_tilde_vec; Fu_tilde_vec]	# Adding state constraints 
    d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0 - F_tilde_vec*E_tilde_vec*g_tilde_vec;  fu_tilde_vec]
    Qdual_tmp = C_dual*(Q_dual\(C_dual'))
    Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))
    
    # L_test = dual_ineq

	# Solve the dual problem online to match cost 
    mdlD = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdlD, L_test[1:N*(nf+ng)])  	# decision variable; contains everything
	@objective(mdlD, Max, -1/2 * L_test'*Qdual_tmp*L_test - (C_dual*(Q_dual\c_dual)+d_dual)'*L_test - 1/2*c_dual'*(Q_dual\c_dual) + const_dual)
	@constraint(mdlD, -L_test .<= 0)

	statusD = solve(mdlD)

	 	if !(statusD == :Optimal)
 			@goto label1 
 		end
	
	inputParam_long[ii,:] = [s_0 v_0 u_0 s_ref v_ref]

	optVal_long[ii] = obj_primal
	solv_time_all[ii] = toq()

	# extract solution
	x_tilde_vec_opt = getvalue(x_tilde_vec)
	dA_pred_opt = getvalue(u_tilde_vec)
	a_pred_opt = x_tilde_vec_opt[3:nx+nu:end]

	## store the primal solution too as output gonna change now 
	outputParamDacc_long[ii,:] =  	dA_pred_opt
	outputParamAcc_long[ii,:]  = 	a_pred_opt
 	###########################################################	

	obj_dualOnline = getobjectivevalue(mdlD)

	# extract solution
	L_test_opt = getvalue(L_test)
	outputParamDual_long[ii,:] = L_test_opt

	dual_gap[ii] = (obj_primal - obj_dualOnline)
	Reldual_gap[ii] = (obj_primal - obj_dualOnline)/obj_primal
# 	#########################################################################

 	ii = ii + 1 

 	@label label1
end


println("===========================================")
# # println("max a-residual:  $(maximum(a_res_all))")
# # println("max dA-residual:  $(maximum(dA_res_all))")
println("max solv-time (excl modelling time):  $(maximum(solv_time_all[3:end]))")
println("avg solv-time (excl modelling time):  $(mean(solv_time_all[3:end]))")
println("max dual_gap:  $(maximum(dual_gap))")
println("min dual_gap:  $(minimum(dual_gap))")
println("max Rel dual_gap:  $(minimum(Reldual_gap))")
println("min Rel dual_gap:  $(minimum(Reldual_gap))")



# ## save data
# # matwrite("NN_test_trainingDataLong_PrimalDual.mat", Dict(
# # 	"inputParam_long" => inputParam_long,
# # 	"outputParamAcc_long" => outputParamAcc_long,
# # 	"outputParamDacc_long" => outputParamDacc_long,
# # 	"outputParamDual_long" => outputParamDual_long,
# # 	"optVal_long" => optVal_long
# # ))
# # println("---- done extracting and saving dual for LONG control ----")




