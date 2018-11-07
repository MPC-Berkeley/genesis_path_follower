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
longData = matread("NN_test_CPGDay3_RandTrainingDataLong10k.mat")   				# bad


inputParam_long = longData["inputParam_long"]   # np.hstack((s_curr.T, v_curr.T ,a_prev.T, s_ref, v_ref ))
# outputParamAcc_long = longData["outputParamAcc_long"]
outputParamDacc_long = longData["outputParamDacc_long"]
outputParamDual_long = longData["outputParamDual_long"]
outputParamOptVal_long = longData["optVal_long"]

###### Merge random data with extra data from vehicle path following    
# inputParam_long =  [s_train_rand v_train_rand aprev_train_rand s_ref_train_rand v_ref_train_rand ]

# s_curr_all = inputParam_long[:,1] 	% always 0
v_curr_all = inputParam_long[:,1]
a_prev_all = inputParam_long[:,2]
s_ref_all =  inputParam_long[:,3:3+N-1]
v_ref_all =  inputParam_long[:,3+N:end]								# All data appended 

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

H_gurobi = kmpcLinLong.H_gurobi
n_uxu = kmpcLinLong.n_uxu
f_gurobi_init = kmpcLinLong.f_gurobi_init
Aeq_gurobi = kmpcLinLong.Aeq_gurobi
ub_gurobi = kmpcLinLong.ub_gurobi
lb_gurobi = kmpcLinLong.lb_gurobi


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

# remove useless/inactive constraints
# F_tilde = F_tilde[[2,3,5,6],:] 		# ignore constraint on s
# f_tilde = f_tilde[[2,3,5,6]]
# nf = length(f_tilde);
nf = 0

# Concatenate appended state (tilde) constraints
# NO input constraints
# F_tilde_vec = kron(eye(N), F_tilde)
# f_tilde_vec = repmat(f_tilde,N)   


######################## ITERATE OVER parameters ################
# build problem
num_DataPoints = size(inputParam_long,1)

solv_time_all = zeros(num_DataPoints)

dA_res_all = zeros(num_DataPoints)
dA_res_all2 = zeros(num_DataPoints)
dual_res_all = zeros(num_DataPoints)

optVal_res_all = zeros(num_DataPoints)
relOptVal_res_all = zeros(num_DataPoints)


dual_gap 	= zeros(num_DataPoints)
Reldual_gap = zeros(num_DataPoints)


beq_gurobi_updated = []


ii=1
while ii <= num_DataPoints
	
	# these are the stored solution
	s_0 = 0 				# set to zero b/c normalized
	v_0 = v_curr_all[ii]
	u_0 = a_prev_all[ii]
	s_ref = s_ref_all[ii,:]
	v_ref = v_ref_all[ii,:]

	dAcc_stored = outputParamDacc_long[ii,:]	
	obj_stored = outputParamOptVal_long[ii]
	dual_stored = outputParamDual_long[ii,:]		

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

	################################################
	# solve primal problem
	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, x_tilde_vec[1:N*(nx+nu)])  	# decision variable; contains everything
	@variable(mdl, u_tilde_vec[1:N*nu] )
	@objective(mdl, Min, (x_tilde_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_vec-x_tilde_ref) + u_tilde_vec'*R_tilde_vec*u_tilde_vec)
	constr_eq = @constraint(mdl, x_tilde_vec .== A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_vec + E_tilde_vec*g_tilde_vec)
	# constr_Fx = @constraint(mdl, F_tilde_vec*x_tilde_vec .<= f_tilde_vec)
	constr_Fu = @constraint(mdl, Fu_tilde_vec*u_tilde_vec .<= fu_tilde_vec)

	status = solve(mdl)

 	if !(status == :Optimal)
		println("****** PROBLEM IN PRIMAL ******")
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
	
	################################################
	######## Trafo 2 ########## 
 	beq_gurobi_updated = repmat(g_tilde,N);
	beq_gurobi_updated[1:nx_tilde] = beq_gurobi_updated[1:nx_tilde] + A_tilde*x_tilde_0 	# PARAMETER: depends on x0
	u_tilde_ref = zeros(N*nu) 	# want to minimize deltaU = u_k - u_{k-1}
	z_gurobi_ref = zeros(N*n_uxu) 	# reference point for z_gurobi ; PARAMETER!
	for i = 1 : N
		z_gurobi_ref[(i-1)*n_uxu+1 : (i-1)*n_uxu+nu] = u_tilde_ref[(i-1)*nu+1 : i*nu] 		# should be zero for this application
		z_gurobi_ref[(i-1)*n_uxu+nu+1 : i*n_uxu] = x_tilde_ref[(i-1)*(nx+nu)+1 : i*(nu+nx)]
	end 
	f_gurobi_updated = -2*H_gurobi*z_gurobi_ref


	mdl2 = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl2, z[1:N*n_uxu])  	# decision variable; contains everything
	@objective(mdl2, Min, z'*H_gurobi*z + f_gurobi_updated'*z)
	constr_eq = @constraint(mdl2, Aeq_gurobi*z .== beq_gurobi_updated)
	constr_ub = @constraint(mdl2, z .<= squeeze(ub_gurobi,2))
	constr_lb = @constraint(mdl2, -z .<= -squeeze(lb_gurobi,2))

	tic()
	status = solve(mdl2)
	if !(status == :Optimal)
		println(status)
		println("PROBLEM IN Z-TRAFO")
 		@goto label1 
 	end


	################################################
	# extract data for comparison
	# comparison of objective
	optVal_res_all[ii] = norm(getobjectivevalue(mdl) - obj_stored)
	relOptVal_res_all[ii] = optVal_res_all[ii] / obj_stored

	# comparison of optimizers
	dA_pred_opt = getvalue(u_tilde_vec)
	dA_res_all[ii] = norm(dA_pred_opt - dAcc_stored)

	z_opt = getvalue(z)
	dA_pred_opt2 = z_opt[1:n_uxu:end]
	dA_res_all2[ii] = norm(dA_pred_opt2 - dAcc_stored)

	L_test_opt = getvalue(L_test)
	dual_res_all[ii] = norm(L_test_opt - dual_stored)

	# sanity checks

	dual_gap[ii] = (getobjectivevalue(mdl) - getobjectivevalue(mdlD))
	Reldual_gap[ii] = dual_gap[ii] / getobjectivevalue(mdl)


	@label label1
 	ii = ii + 1 

end

println("===========================================")
println("max optVal residual: $(maximum(optVal_res_all))")
println("min optVal residual: $(minimum(optVal_res_all))")
println("avg optVal residual: $(mean(optVal_res_all))")

println(" ")

println("max rel optVal residual: $(maximum(relOptVal_res_all))")
println("min rel optVal residual: $(minimum(relOptVal_res_all))")
println("avg rel optVal residual: $(mean(relOptVal_res_all))")

println(" ")

println("max dA-residual:  $(maximum(dA_res_all))")
println("max dA-residual2: $(maximum(dA_res_all2))")
println("max dual-residual: $(maximum(dual_res_all))")


println(" ")


println("max dual_gap:  $(maximum(dual_gap))")
println("min dual_gap:  $(minimum(dual_gap))")
println("avg dual_gap:  $(mean(dual_gap))")

println(" ")

println("max Rel dual_gap:  $(minimum(Reldual_gap))")
println("min Rel dual_gap:  $(minimum(Reldual_gap))")
println("avg Rel dual_gap:  $(mean(Reldual_gap))")



#save data
# matwrite("NN_test_trainingDataLong10k_RegDual1e-7.mat", Dict(
# 	"inputParam_long" => inputParam_long,
# 	"outputParamAcc_long" => outputParamAcc_long,
# 	"outputParamDacc_long" => outputParamDacc_long,
# 	"outputParamDual_long" => outputParamDual_long,
# 	"optVal_long" => optVal_long
# ))
# println("---- done extracting and saving dual for LONG control ----")




