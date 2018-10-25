# code to test and extract dual multipliers from primal solution
# GXZ + MB

using MAT
using Gurobi
using JuMP


## Code for concatinating matrices 

# function appendMat(N, nx, nu, nw, A, B, E)

# 	Ax_vec = zeros(N*nx, nx)
#         for ii = 1 : N
#             Ax_vec[1+(ii-1)*nx:ii*nx,:] = A^ii
#         end

# 	Bx_vec = zeros(N*nx, nu*N)

#         for ii = 0 : N-1
#             for jj = 0 : ii-1
#                 Bx_vec[1+ii*nx:(ii+1)*nx, 1+jj*nu:  (jj+1)*nu] = A^(ii-jj)*B
#             end
#             Bx_vec[1+ii*nx:(ii+1)*nx, 1+ii*nu:(ii+1)*nu] = B
#         end

#     Ex_vec = zeros(N*nx, nw*N)

#         for ii = 0 : N-1
#             for jj = 0 : ii-1
#                 Ex_vec[1+ii*nx:(ii+1)*nx, 1+jj*nw:  (jj+1)*nw] = A^(ii-jj)*E
#             end
#             Ex_vec[1+ii*nx:(ii+1)*nx, 1+ii*nw:(ii+1)*nw] = E
#         end

# 	return Ax_vec, Bx_vec, Ex_vec
# end


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

B_tilde = kmpcLinLong.B_tilde

g_tilde = kmpcLinLong.g_tilde

x_tilde_lb = kmpcLinLong.x_tilde_lb
x_tilde_ub = kmpcLinLong.x_tilde_ub
u_tilde_lb = kmpcLinLong.u_tilde_lb
u_tilde_ub = kmpcLinLong.u_tilde_ub

Q_tilde = kmpcLinLong.Q_tilde
# Q_tilde = [	Q 			zeros(nx,nu) 		# may also use cat(?)
			# zeros(nu,nx)	R 			]	# actually not needed

R_tilde = kmpcLinLong.R_tilde
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

Q_tilde_vec = kron(eye(N),Q_tilde)
R_tilde_vec = kron(eye(N),R_tilde)

# A_tilde_vec, B_tilde_vec, E_tilde_vec = appendMat(N, nx, nu, A_tilde, B_tilde, eye(nx+nu))
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
num_DataPoints = length(s_curr_all)
solv_time_all = zeros(num_DataPoints)
a_res_all = zeros(num_DataPoints)
dA_res_all = zeros(num_DataPoints)
dual_gap = zeros(num_DataPoints)

outputParamDual_long = zeros(num_DataPoints, N*(nf+ng))

# dual_eq = []
# dual_ub = []
# dual_lb = []
# z_opt_uc = []
# opt_val_dual = []
dual_Fx = []
dual_Fu = []
L_test_opt = []

for ii = 1 : num_DataPoints
	# extract appropriate parameters
	s_0 = s_curr_all[ii]
	v_0 = v_curr_all[ii]
	u_0 = a_prev_all[ii]
	s_ref = s_ref_all[ii,:]
	v_ref = v_ref_all[ii,:]
	acc_stored = outputParamAcc_long[ii,:]
	dAcc_stored = outputParamDacc_long[ii,:]

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
################################################### TRAFO 1


	x0 = [s_0 ; v_0]
	u0 = u_0 				# it's really u_{-1}
	x_tilde_0 = [x0 ; u0]	# initial state of system; PARAMETER


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
	solv_time_all[ii] = toq()

	# extract solution
	x_tilde_vec_opt = getvalue(x_tilde_vec)
	dA_pred_opt = getvalue(u_tilde_vec)
	a_pred_opt = x_tilde_vec_opt[3:nx+nu:end]
	# s_pred_opt = z_opt[2:n_uxu:end]  	# does not include s0
	# v_pred_opt = z_opt[3:n_uxu:end] 		# does not include v0 

	#### compare solution ####
	a_res_all[ii] = norm(a_pred_opt - acc_stored)
	dA_res_all[ii] = norm(dA_pred_opt - dAcc_stored)

	#### extract dual variables ####
	dual_eq = getdual(constr_eq)
	dual_Fx = getdual(constr_Fx)
	dual_Fu = getdual(constr_Fu)

	dual_ineq = [dual_Fx; dual_Fu]


	## compute dual cost
	Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
     
    c_dual = (2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*B_tilde_vec +
    	      - 2*x_tilde_ref'*Q_tilde_vec*B_tilde_vec)'
     
    const_dual = x_tilde_0'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0 + 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  + g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  - 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref - 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref +
                  + x_tilde_ref'*Q_tilde_vec*x_tilde_ref
        
    C_dual = [F_tilde_vec*B_tilde_vec; Fu_tilde_vec]		        # Adding state constraints 
    d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0 - F_tilde_vec*E_tilde_vec*g_tilde_vec;  fu_tilde_vec]
    Qdual_tmp = C_dual*(Q_dual\(C_dual'))
    Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))
    
    # L_test = dual_ineq

	# Solve the dual problem online to match cost 
    mdlD = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdlD, L_test[1:N*(nf+ng)])  	# decision variable; contains everything
	@objective(mdlD, Max, -1/2 * L_test'*Qdual_tmp*L_test - (C_dual*(Q_dual\c_dual)+d_dual)'*L_test - 1/2*c_dual'*(Q_dual\c_dual) + const_dual)
	@constraint(mdlD, -L_test .<= 0)
	# @constraint(mdlD, L_test .== -dual_ineq)

	statusD = solve(mdlD)
	obj_dualOnline = getobjectivevalue(mdlD)

	# extract solution
	L_test_opt = getvalue(L_test)
	outputParamDual_long[ii,:] = L_test_opt


	dual_gap[ii] = (obj_primal - obj_dualOnline)
	#########################################################################

end

println("max a-residual:  $(maximum(a_res_all))")
println("max dA-residual:  $(maximum(dA_res_all))")
println("max solv-time (excl modelling time):  $(maximum(solv_time_all[3:end]))")
println("avg solv-time (excl modelling time):  $(mean(solv_time_all[3:end]))")
println("max dual_gap:  $(maximum(dual_gap))")
println("min dual_gap:  $(minimum(dual_gap))")


# save data
# matwrite("NN_test_trainingDataLong_PrimalDual.mat", Dict(
# 	"inputParam_long" => inputParam_long,
# 	"outputParamAcc_long" => outputParamAcc_long,
# 	"outputParamDacc_long" => outputParamDacc_long,
# 	"outputParamDual_long" => outputParamDual_long,
# ))


println("---- done extracting and saving dual for LONG control ----")




