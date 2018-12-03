# Code to test the quality of the trained nets for Lateral control
# GXZ + MB
# From real data. That's why we must solve the optimization problem here. 

using MAT
using Gurobi
using JuMP


#### problem paramters for LATERAL Control 
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

############## load all NN Matrices ##############
primalNN_Data 	= matread("latNNandData/trained_weightsPrimalLatBadRand10kData_OneTraj_CPGDay5.mat")
# dualNN_Data 	= matread("trained_weightsDualLatBadVCRand10kDataTwoTraj_CPGDay4.mat")		 

# read out NN primal/Dual weights
Wi_PLat = primalNN_Data["W1"]
bi_PLat = primalNN_Data["b1"]
W1_PLat = primalNN_Data["W2"]
b1_PLat = primalNN_Data["b2"]
Wout_PLat = primalNN_Data["W0"]
bout_PLat = primalNN_Data["b0"]

##

# Wi_DLat = dualNN_Data["W1"]
# bi_DLat = dualNN_Data["b1"]
# W1_DLat = dualNN_Data["W2"]
# b1_DLat = dualNN_Data["b2"]
# Wout_DLat = dualNN_Data["W0"]
# bout_DLat = dualNN_Data["b0"]

####################### debugging code ###################################
test_Data = matread("CPG_day5_lat1B_trainingData.mat")
test_inputParams = test_Data["inputParam_lat"]
# test_optVal = test_Data["optVal_lat"]
test_Ddf = test_Data["outputParamDdf_lat"]
# test_dual = test_Data["outputParamDual_lat"]
test_inputParams = test_inputParams[50:1550,:]
num_DataPoints = size(test_inputParams,1)

###########################################################################
# ================== Transformation 1 =======================
# augment state and redefine system dynamics (A,B,g) and constraints
# x_tilde_k := (x_k , u_{k-1})
# u_tilde_k := (u_k - u_{k-1})
# Load all the required stuff 

x_tilde_lb = kmpcLinLat.x_tilde_lb
x_tilde_ub = kmpcLinLat.x_tilde_ub
u_tilde_lb = kmpcLinLat.u_tilde_lb
u_tilde_ub = kmpcLinLat.u_tilde_ub

Q_tilde = kmpcLinLat.Q_tilde
R_tilde = kmpcLinLat.R_tilde

nu_tilde = kmpcLinLat.nu_tilde
nx_tilde = kmpcLinLat.nx_tilde

Q_tilde_vec = kron(eye(N),Q_tilde)   								# for x_tilde_vec
R_tilde_vec = kron(eye(N),R_tilde)	 								# for u_tilde_vec

u_ref_init = kmpcLinLat.u_ref_init									# if not used, set cost to zeros
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

######################## ITERATE OVER parameters ################
# build problem
# num_DataPoints = 1000						# Number of test data points
num_DataPoints = size(test_inputParams,1)

gap_primal 				= zeros(num_DataPoints)
relGap_primal 			= zeros(num_DataPoints)
gap_dual 				= zeros(num_DataPoints)
relGap_dual 			= zeros(num_DataPoints)
gap_primalNNdualNN 		= zeros(num_DataPoints)
relGap_primalNNdualNN 	= zeros(num_DataPoints)

ddf_res 	= zeros(num_DataPoints)
lambda_res 	= zeros(num_DataPoints)

numbSkipped = 0
flag_XUfeas = 0

x_tilde_ref = x_tilde_ref_init

iii = 1


while iii <= num_DataPoints
	
	if mod(iii,100) == 0
		println("iteration: $(iii)")
	end

	params = test_inputParams[iii,:]
	v_pred = params[4:4+N-1]
	c_pred = params[4+N:end]
	# optVal_saved = test_optVal[iii]

	
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
						-dt*v_pred[i].*c_pred[i]]	# param merge 
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

	# println("--- Differences E_tilde_vec and E_tilde_vec2: $(norm(E_tilde_vec - E_tilde_vec2))")

	E_tilde_vec = E_tilde_vec2
	B_tilde_vec = B_tilde_vec2

	g_tilde_vec = zeros(N*(nx+nu))
	for ii = 1 : N
		g_tilde_vec[1+(ii-1)*(nx+nu) : ii*(nx+nu)] = g_tilde_updated[:,ii]
	end

	################## BEGIN extract Primal NN solution ##################
	tic()
	# calls the NN with two Hidden Layers
	z1 = max.(Wi_PLat*params + bi_PLat, 0)
	z2 = max.(W1_PLat*z1 + b1_PLat, 0)
	u_tilde_NN_vec = Wout_PLat*z2 + bout_PLat 
	# do projection
	u_tilde_NN_vec = min.(u_tilde_NN_vec, u_tilde_ub)
	u_tilde_NN_vec = max.(u_tilde_NN_vec, u_tilde_lb)

	##################################################
	
	# compute NN predicted state
	x_tilde_0 = params[1:3] 	
	x_tilde_NN_vec = A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_NN_vec + E_tilde_vec*g_tilde_vec

	## verify feasibility
	# xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
	xu_tilde_NN_res = maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec)  # should be <= 0

	if maximum(xu_tilde_NN_res) < 1e-3  						# infeasible if bigger than zero/threshold
		flag_XUfeas = flag_XUfeas+1
	end

	## check optimality ##
	# primObj_NN = (x_tilde_NN_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_NN_vec-x_tilde_ref) + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec
	primObj_NN = x_tilde_NN_vec'*Q_tilde_vec*x_tilde_NN_vec + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec
	solvTime_NN = toq()
	################## END extract Primal NN solution ##################

	
	################## BEGIN extract Dual NN solution ##################
	
	# Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
     
 #    c_dual = (2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*B_tilde_vec +
 #    	      - 2*x_tilde_ref'*Q_tilde_vec*B_tilde_vec)'
     
 #    const_dual = x_tilde_0'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0 + 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
 #                  + g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
 #                  - 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref - 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref +
 #                  + x_tilde_ref'*Q_tilde_vec*x_tilde_ref
        
 #    # C_dual = [F_tilde_vec*B_tilde_vec; Fu_tilde_vec]		        # Adding state constraints 
 #    C_dual = Fu_tilde_vec		        # Adding state constraints 
 #    # d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0 - F_tilde_vec*E_tilde_vec*g_tilde_vec;  fu_tilde_vec]
 #    d_dual = fu_tilde_vec
 #    Qdual_tmp = C_dual*(Q_dual\(C_dual'))
 #    Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))


	# # calls the NN with two Hidden Layers
	# z1D = max.(Wi_DLat*params + bi_DLat, 0)
	# z2D = max.(W1_DLat*z1D + b1_DLat, 0)
	# lambda_tilde_NN_orig = Wout_DLat*z2D + bout_DLat
	# # need projection
	# lambda_tilde_NN_vec = max.(lambda_tilde_NN_orig, 0)  	#Delta-Acceleration

	# dualObj_NN = -1/2 * lambda_tilde_NN_vec'*Qdual_tmp*lambda_tilde_NN_vec - (C_dual*(Q_dual\c_dual)+d_dual)'*lambda_tilde_NN_vec - 1/2*c_dual'*(Q_dual\c_dual) + const_dual
	
	################## END extract Dual NN solution ##################

	# solve primal problem
	# use it to test consistency
	mdl = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdl, x_tilde_vec[1:N*(nx+nu)])  	# decision variable; contains everything
	@variable(mdl, u_tilde_vec[1:N*nu] )
	@objective(mdl, Min, (x_tilde_vec)'*Q_tilde_vec*(x_tilde_vec) + u_tilde_vec'*R_tilde_vec*u_tilde_vec)
	constr_eq = @constraint(mdl, x_tilde_vec .== A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_vec + E_tilde_vec*g_tilde_vec)
	# constr_Fx = @constraint(mdl, F_tilde_vec*x_tilde_vec .<= f_tilde_vec)
	constr_Fu = @constraint(mdl, Fu_tilde_vec*u_tilde_vec .<= fu_tilde_vec)

	tic()
	status = solve(mdl)
 	if !(status == :Optimal)
 		numbSkipped = numbSkipped+1
 		@goto label1 
 	end

 	obj_primal = getobjectivevalue(mdl)


	ddf_res[iii] = norm(test_Ddf[iii,:] - u_tilde_NN_vec)
	# lambda_res[iii] = norm(test_dual[iii,:] - lambda_tilde_NN_vec)
	
	gap_primal[iii] = primObj_NN[1] - obj_primal
	relGap_primal[iii] = gap_primal[iii] / obj_primal

	# gap_dual[iii] = obj_primal - dualObj_NN[1]
	# relGap_dual[iii] = gap_dual[iii] / obj_primal

	# gap_primalNNdualNN[iii] = primObj_NN[1] - dualObj_NN[1]
	# relGap_primalNNdualNN[iii] = gap_primalNNdualNN[iii] / obj_primal

 	iii = iii + 1 

 	@label label1
end

println("$(flag_XUfeas)")

println("===========================================")


println("max ddf res:  $(maximum(ddf_res))")
println("min ddf res:  $(minimum(ddf_res))")
println("avg ddf res:  $(mean(ddf_res))")

# println(" ")

# println("max lambda res:  $(maximum(lambda_res))")
# println("min lambda res:  $(minimum(lambda_res))")
# println("avg lambda res:  $(mean(lambda_res))")

println(" ")


println("max primal NN gap:  $(maximum(gap_primal))")
println("min primal NN gap:  $(minimum(gap_primal))")
println("avg primal NN gap:  $(mean(gap_primal))")

println(" ")

println("max rel primal NN gap:  $(maximum(relGap_primal))")
println("min rel primal NN gap:  $(minimum(relGap_primal))")
println("avg rel primal NN gap:  $(mean(relGap_primal))")

# println(" ")

# println("max dual NN gap:  $(maximum(gap_dual))")
# println("min dual NN gap:  $(minimum(gap_dual))")
# println("avg dual NN gap:  $(mean(gap_dual))")

# println(" ")

# println("max rel dual NN gap:  $(maximum(relGap_dual))")
# println("min rel dual NN gap:  $(minimum(relGap_dual))")
# println("avg rel dual NN gap:  $(mean(relGap_dual))")

# println(" ")

# println("max primal-dual NN gap:  $(maximum(gap_primalNNdualNN))")
# println("min primal-dual NN gap:  $(minimum(gap_primalNNdualNN))")
# println("avg primal-dual NN gap:  $(mean(gap_primalNNdualNN))")

# println(" ")

# println("max rel primal-dual NN gap:  $(maximum(relGap_primalNNdualNN))")
# println("min rel primal-dual NN gap:  $(minimum(relGap_primalNNdualNN))")
# println("avg rel primal-dual NN gap:  $(mean(relGap_primalNNdualNN))")

