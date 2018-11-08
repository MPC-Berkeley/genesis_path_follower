# Code to test the quality of the trained nets for Longitudinal control
# GXZ + MB
# This one just matches stored stuff. That's why the stored data loaded must be 
# processed data containing optVal and Duals. The other two test codes actually
# just need the params. They solve raw data, generate stuff and then match and 
# test NN quality. 

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

############## load all NN Matrices ##############
primalNN_Data 	= matread("trained_weightsPrimalLongTotalData_CPGDay4.mat")
dualNN_Data 	= matread("trained_weightsDualLongTotalData_CPGDay4.mat")

# read out NN primal/Dual weights
Wi_PLong = primalNN_Data["W1"]
bi_PLong = primalNN_Data["b1"]
W1_PLong = primalNN_Data["W2"]
b1_PLong = 	primalNN_Data["b2"]
Wout_PLong = primalNN_Data["W0"]
bout_PLong = primalNN_Data["b0"]

Wi_DLong = dualNN_Data["W1"]
bi_DLong = dualNN_Data["b1"]
W1_DLong = dualNN_Data["W2"]
b1_DLong = dualNN_Data["b2"]
Wout_DLong = dualNN_Data["W0"]
bout_DLong = dualNN_Data["b0"]

####################### debugging code ###################################
test_Data = matread("NN_test_CPGDay3_TotalDataLong_complete.mat")
test_inputParams = test_Data["inputParam_long"]
test_optVal = test_Data["optVal_long"]
test_Dacc = test_Data["outputParamDacc_long"]
test_dual = test_Data["outputParamDual_long"]
# test_inputParams = test_inputParams[1:1000,:]

############################################################################

## Load Ranges of params 
# input reference
u_ref_init = kmpcLinLong.u_ref_init		# if not used, set cost to zeros

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
# F_tilde = [eye(nx+nu) ; -eye(nx+nu)]
# f_tilde = [x_tilde_ub ; -x_tilde_lb]
# nf = length(f_tilde)
nf = 0

# Concatenate appended state (tilde) constraints
# F_tilde_vec = kron(eye(N), F_tilde)
# f_tilde_vec = repmat(f_tilde,N)   

Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
C_dual = Fu_tilde_vec				# Adding state constraints 
Qdual_tmp = C_dual*(Q_dual\(C_dual'))
Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))

######################## ITERATE OVER parameters ################
# build problem
num_DataPoints = size(test_inputParams,1)
# num_DataPoints = 1e3

gap_primal = zeros(num_DataPoints)
relGap_primal = zeros(num_DataPoints)
gap_dual = zeros(num_DataPoints)
relGap_dual = zeros(num_DataPoints)
gap_primalNNdualNN = zeros(num_DataPoints)
relGap_primalNNdualNN = zeros(num_DataPoints)

dAcc_res = zeros(num_DataPoints)
lambda_res = zeros(num_DataPoints)

numbSkipped = 0
flag_XUfeas = 0

# dualObj_NN = -1/2 * lambda_tilde_NN_vec'*Qdual_tmp*lambda_tilde_NN_vec - (C_dual*(Q_dual\c_dual)+d_dual)'*lambda_tilde_NN_vec - 1/2*c_dual'*(Q_dual\c_dual) + const_dual

ii = 1
while ii <= num_DataPoints
	
	if mod(ii,100) == 0
		println("****************** iteration: $(ii) ***********************")
	end

############# for debugging ###############
	params = test_inputParams[ii,:]
	s_ref = params[3:3+N-1]
	v_ref = params[3+N:end]
	optVal_saved = test_optVal[ii]

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

	################## BEGIN extract Primal NN solution ##################
	tic()
	# calls the NN with two Hidden Layers
	z1 = max.(Wi_PLong*params + bi_PLong, 0)
	z2 = max.(W1_PLong*z1 + b1_PLong, 0)
	u_tilde_NN_vec = Wout_PLong*z2 + bout_PLong  	#Delta-Acceleration

	# do projection
	u_tilde_NN_vec = min.(u_tilde_NN_vec, u_tilde_ub)
	u_tilde_NN_vec = max.(u_tilde_NN_vec, u_tilde_lb)

	# compute NN predicted state
	x_tilde_0 = [0 ; params[1:2]] 	
	x_tilde_NN_vec = A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_NN_vec + E_tilde_vec*g_tilde_vec

	## verify feasibility
	# xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*x_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
	# xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
	# flag_XUfeas = 0
	xu_tilde_NN_res = maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec)

	if maximum(xu_tilde_NN_res) < 1e-3  	# infeasible if bigger than zero/threshold
		# numbSkipped = numbSkipped + 1
		flag_XUfeas = flag_XUfeas + 1
	end

	## check optimality ##
	primObj_NN = (x_tilde_NN_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_NN_vec-x_tilde_ref) + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec
	solvTime_NN = toq()

	a_opt_NN = x_tilde_NN_vec[3]
	a_pred_NN = x_tilde_NN_vec[3:(nx+nu):end]
	s_pred_NN = x_tilde_NN_vec[1:(nx+nu):end]
	v_pred_NN = x_tilde_NN_vec[2:(nx+nu):end]
	
	################## END extract Primal NN solution ##################

	################## BEGIN extract Dual NN solution ##################
	
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
	z1D = max.(Wi_DLong*params + bi_DLong, 0)
	z2D = max.(W1_DLong*z1D + b1_DLong, 0)
	lambda_tilde_NN_orig = Wout_DLong*z2D + bout_DLong

	# need to project
	lambda_tilde_NN_vec = max.(lambda_tilde_NN_orig, 0)  	#Delta-Acceleration
	# lambda_tilde_NN_vec = lambda_tilde_NN_orig
	
	dualObj_NN = -1/2 * lambda_tilde_NN_vec'*Qdual_tmp*lambda_tilde_NN_vec - (C_dual*(Q_dual\c_dual)+d_dual)'*lambda_tilde_NN_vec - 1/2*c_dual'*(Q_dual\c_dual) + const_dual
	# dualObj_NN = -1/2 * lambda_tilde_NN_vec'*Qdual_tmp*lambda_tilde_NN_vec - (C_dual*(Q_dual\c_dual)+d_dual)'*lambda_tilde_NN_vec - 1/2*c_dual'*(Q_dual\c_dual) + const_dual
	################## BEGIN extract Dual NN solution ##################

	# solve primal problem
	# use it to test consistency
	# mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	# @variable(mdl, x_tilde_vec[1:N*(nx+nu)])  	# decision variable; contains everything
	# @variable(mdl, u_tilde_vec[1:N*nu] )
	# @objective(mdl, Min, (x_tilde_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_vec-x_tilde_ref) + u_tilde_vec'*R_tilde_vec*u_tilde_vec)
	# constr_eq = @constraint(mdl, x_tilde_vec .== A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_vec + E_tilde_vec*g_tilde_vec)
	# constr_Fx = @constraint(mdl, F_tilde_vec*x_tilde_vec .<= f_tilde_vec)
	# constr_Fu = @constraint(mdl, Fu_tilde_vec*u_tilde_vec .<= fu_tilde_vec)

	# tic()
	# status = solve(mdl)
	# obj_primal = getobjectivevalue(mdl)
	# U_test_opt = getvalue(u_tilde_vec)
	# primalDiff[ii] = norm(U_test_opt - u_tilde_NN_vec)
	# primalDiff0[ii] = norm(U_test_opt[1] - u_tilde_NN_vec[1])


 # 	if !(status == :Optimal)
 # 		ii = ii + 1
 # 		numbSkipped = numbSkipped+1
 # 		@goto label1 
 # 	end


 	### for debugging purposes; 
 	# reg 1e-8: 
 	# reg 1e-7: 800 out of 8000 skipped, max dual gap 2.6; max rel gap 0.0001
 	# reg 1e-6: 800 out of 8000 skipped, max dual gap: 28; max rel gap: 0.001
 	# reg 1e-5: 
	# Solve the dual problem online to match cost 
 #    mdlD = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	# @variable(mdlD, L_test[1:N*(nf+ng)])  	# decision variable; contains everything
	# @objective(mdlD, Max, -1/2 * L_test'*Qdual_tmp*L_test - (C_dual*(Q_dual\c_dual)+d_dual)'*L_test - 1/2*c_dual'*(Q_dual\c_dual) + const_dual - 1e-7*L_test'eye(N*(nf+ng))*L_test )
	# @constraint(mdlD, -L_test .<= 0)
	# statusD = solve(mdlD)
	# obj_dualOnline = getobjectivevalue(mdlD)
	# L_test_opt = getvalue(L_test)

 # 	if !(statusD == :Optimal)
 # 		ii = ii + 1
 # 		@goto label1 
 # 	end


	dAcc_res[ii] = norm(test_Dacc[ii,:] - u_tilde_NN_vec)
	lambda_res[ii] = norm(test_dual[ii,:] - lambda_tilde_NN_vec)
	

	gap_primal[ii] = primObj_NN[1] - optVal_saved
	relGap_primal[ii] = gap_primal[ii] / optVal_saved

	gap_dual[ii] = optVal_saved - dualObj_NN[1]
	relGap_dual[ii] = gap_dual[ii] / optVal_saved

	gap_primalNNdualNN[ii] = primObj_NN[1] - dualObj_NN[1]
	relGap_primalNNdualNN[ii] = gap_primalNNdualNN[ii] / optVal_saved


 	ii = ii + 1 

 	@label label1
end

println("$(flag_XUfeas)")

println("===========================================")


println("max dACC res:  $(maximum(dAcc_res))")
println("min dACC res:  $(minimum(dAcc_res))")
println("avg dACC res:  $(mean(dAcc_res))")

println(" ")

println("max lambda res:  $(maximum(lambda_res))")
println("min lambda res:  $(minimum(lambda_res))")
println("avg lambda res:  $(mean(lambda_res))")

println(" ")


println("max primal NN gap:  $(maximum(gap_primal))")
println("min primal NN gap:  $(minimum(gap_primal))")
println("avg primal NN gap:  $(mean(gap_primal))")

println(" ")

println("max rel primal NN gap:  $(maximum(relGap_primal))")
println("min rel primal NN gap:  $(minimum(relGap_primal))")
println("avg rel primal NN gap:  $(mean(relGap_primal))")

println(" ")

println("max dual NN gap:  $(maximum(gap_dual))")
println("min dual NN gap:  $(minimum(gap_dual))")
println("avg dual NN gap:  $(mean(gap_dual))")

println(" ")

println("max rel dual NN gap:  $(maximum(relGap_dual))")
println("min rel dual NN gap:  $(minimum(relGap_dual))")
println("avg rel dual NN gap:  $(mean(relGap_dual))")

println(" ")

println("max primal-dual NN gap:  $(maximum(gap_primalNNdualNN))")
println("min primal-dual NN gap:  $(minimum(gap_primalNNdualNN))")
println("avg primal-dual NN gap:  $(mean(gap_primalNNdualNN))")

println(" ")

println("max rel primal-dual NN gap:  $(maximum(relGap_primalNNdualNN))")
println("min rel primal-dual NN gap:  $(minimum(relGap_primalNNdualNN))")
println("avg rel primal-dual NN gap:  $(mean(relGap_primalNNdualNN))")

