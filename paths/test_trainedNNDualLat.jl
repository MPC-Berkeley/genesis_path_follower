# Code to test the quality of the trained nets for Lateral control
# GXZ + MB

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
primalNN_Data 	= matread("trained_weightsPrimalLat100k_CPGDay2BacktoDay1Tune.mat")
dualNN_Data 	= matread("trained_weightsDualLat.mat")		 

# read out NN primal/Dual weights
Wi_PLat = primalNN_Data["W1"]
bi_PLat = primalNN_Data["b1"]
W1_PLat = primalNN_Data["W2"]
b1_PLat = primalNN_Data["b2"]
Wout_PLat = primalNN_Data["W0"]
bout_PLat = primalNN_Data["b0"]

##

Wi_DLat = dualNN_Data["W1D"]
bi_DLat = dualNN_Data["b1D"]
W1_DLat = dualNN_Data["W2D"]
b1_DLat = dualNN_Data["b2D"]
Wout_DLat = dualNN_Data["W0D"]
bout_DLat = dualNN_Data["b0D"]


####################### debugging code ###################################
test_Data = matread("NN_test_CPGDay2BacktoDay1Tune_RandDataLat10kTrafo2.mat")
# test_Data = matread("NN_test_trainingDataLat10k_PrimalDual2.mat")
test_inputParams = test_Data["inputParam_lat"]
test_outputParamDdf = test_Data["outputParamDdf_lat"]
# test_inputParams = test_inputParams[50:1550,:]


num_DataPoints = size(test_inputParams,1)

###########################################################################

## Load Ranges of params 
ey_lb = KinMPCParams.ey_lb
ey_ub = KinMPCParams.ey_ub
epsi_lb = KinMPCParams.epsi_lb
epsi_ub = KinMPCParams.epsi_ub
dfprev_lb = -KinMPCParams.df_max
dfprev_ub =  KinMPCParams.df_max

v_lb = KinMPCParams.v_min 
v_ub = KinMPCParams.v_max
curv_lb = KinMPCParams.curv_lb
curv_ub = KinMPCParams.curv_ub

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
F_tilde = [eye(nx+nu) ; -eye(nx+nu)]
f_tilde = [x_tilde_ub ; -x_tilde_lb]
nf = length(f_tilde);

# Concatenate appended state (tilde) constraints
F_tilde_vec = kron(eye(N), F_tilde)
f_tilde_vec = repmat(f_tilde,N)   

    
######################## ITERATE OVER parameters ################
# build problem
# num_DataPoints = 1000						# Number of test data points

solv_time_all = zeros(num_DataPoints)
dual_gap = zeros(num_DataPoints)
Reldual_gap = zeros(num_DataPoints)
PrimandOnline_gap = zeros(num_DataPoints)
RelPrimandOnline_gap = zeros(num_DataPoints)
optVal_lat = zeros(num_DataPoints)
DualOnline_gap = zeros(num_DataPoints)
RelDualOnline_gap = zeros(num_DataPoints)


#### vectors for debugging ####
dualDiff = zeros(num_DataPoints)
primalDiff = zeros(num_DataPoints)
primalDiff0 = zeros(num_DataPoints)
primalDiffOrigSol = zeros(num_DataPoints)


dual_Fx = []
dual_Fu = []
L_test_opt = []

iii = 1

x_tilde_ref = x_tilde_ref_init

while iii <= num_DataPoints
	
	if mod(iii,100) == 0
		println("iteration: $(iii)")
	end

	# Save only feasible points. 
	# extract appropriate parameters	
 	ey_0 = ey_lb + (ey_ub-ey_lb)*rand(1)				
 	epsi_0 = epsi_lb + (epsi_ub-epsi_lb)*rand(1) 
 	u_0 = dfprev_lb + (dfprev_ub-dfprev_lb)*rand(1) 		
	v_pred = v_lb + (v_ub-v_lb)*rand(1,N)						#  Along horizon 
	c_pred = curv_lb + (curv_ub-curv_lb)*rand(1,N)				#  Along horizon 

 	# stack everything together
	params = [ey_0  epsi_0  u_0  v_pred  c_pred]' 				# stack to 19x1 matrix

	# load stuff
	params = test_inputParams[iii,:]
	v_pred = params[4:4+N-1]
	c_pred = params[12:end]

	
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
	z0 = params	
	# z0 = (params - xinoff).*xingain + xinymin
	z1 = max.(Wi_PLat*z0 + bi_PLat, 0)
	z2 = max.(W1_PLat*z1 + b1_PLat, 0)

	u_tilde_NN_vec = Wout_PLat*z2 + bout_PLat 
	
	# compute NN predicted state
	x_tilde_0 = params[1:3] 	
	x_tilde_NN_vec = A_tilde_vec*x_tilde_0 + B_tilde_vec*u_tilde_NN_vec + E_tilde_vec*g_tilde_vec

	## verify feasibility
	# xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*x_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
	xu_tilde_NN_res = [ maximum(F_tilde_vec*x_tilde_NN_vec - f_tilde_vec) ; maximum(Fu_tilde_vec*u_tilde_NN_vec - fu_tilde_vec) ]  # should be <= 0
	flag_XUfeas = 0

	if maximum(xu_tilde_NN_res) < 1e-3  						# infeasible if bigger than zero/threshold
		flag_XUfeas = 1
	end

	## check optimality ##
	# primObj_NN = (x_tilde_NN_vec-x_tilde_ref)'*Q_tilde_vec*(x_tilde_NN_vec-x_tilde_ref) + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec
	primObj_NN = x_tilde_NN_vec'*Q_tilde_vec*x_tilde_NN_vec + u_tilde_NN_vec'*R_tilde_vec*u_tilde_NN_vec
	solvTime_NN = toq()

	################## END extract Primal NN solution ##################

	# dualNN_obj, lambda_tilde_NN_vec = eval_DualNN(params)
	
	################## BEGIN extract Dual NN solution ##################
	
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


	# calls the NN with two Hidden Layers
	z1D = max.(Wi_DLat*params + bi_DLat, 0)
	z2D = max.(W1_DLat*z1D + b1_DLat, 0)
	lambda_tilde_NN_orig = Wout_DLat*z2D + bout_DLat
	lambda_tilde_NN_vec = max.(Wout_DLat*z2D + bout_DLat, 0)  	#Delta-Acceleration

	dualObj_NN = -1/2 * lambda_tilde_NN_vec'*Qdual_tmp*lambda_tilde_NN_vec - (C_dual*(Q_dual\c_dual)+d_dual)'*lambda_tilde_NN_vec - 1/2*c_dual'*(Q_dual\c_dual) + const_dual
	
	################## BEGIN extract Dual NN solution ##################

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
	U_test_opt = getvalue(u_tilde_vec)
	# primalDiff[iii] = norm(U_test_opt - u_tilde_NN_vec)
	# primalDiff0[iii] = norm(U_test_opt[1] - u_tilde_NN_vec[1]) 
	primalDiff[iii] = norm(test_outputParamDdf[iii,:] - u_tilde_NN_vec)
	primalDiff0[iii] = norm(test_outputParamDdf[iii,1] - u_tilde_NN_vec[1]) 
	primalDiffOrigSol[iii] = norm(U_test_opt - test_outputParamDdf[iii,:])

	
	if !(status == :Optimal)
		println(status)
		primStatusError = primStatusError+1
		@goto label1
	end

	optVal_lat[iii] = obj_primal
	solv_time_all[iii] = toq()
	
	PrimandOnline_gap[iii] = primObj_NN[1] - obj_primal
	RelPrimandOnline_gap[iii] = (primObj_NN[1] - obj_primal)/obj_primal
 # 	###########################################################	

	DualOnline_gap[iii] = obj_primal - dualObj_NN[1]
	RelDualOnline_gap[iii] = (obj_primal - dualObj_NN[1])/obj_primal
 # 	###########################################################	

	dual_gap[iii] = primObj_NN[1] - dualObj_NN[1]
	Reldual_gap[iii] = (primObj_NN[1] - dualObj_NN[1])/obj_primal

 	iii = iii + 1 

 	@label label1
end


println("===========================================")
# println("max dual_gap:  $(maximum(dual_gap))")
# println("min dual_gap:  $(minimum(dual_gap))")
# println("max Rel dual_gap:  $(maximum(Reldual_gap))")
# println("min Rel dual_gap:  $(minimum(Reldual_gap))")
# println("avg Rel dual_gap:  $(mean(Reldual_gap))")

println("max onlineNN_gap:  $(maximum(PrimandOnline_gap))")
println("min onlineNN_gap:  $(minimum(PrimandOnline_gap))")
println("avr onlineNN_gap:  $(mean(PrimandOnline_gap))")

println(" ")

println("max Rel onlineNN_gap:  $(maximum(RelPrimandOnline_gap))")
println("min Rel onlineNN_gap:  $(minimum(RelPrimandOnline_gap))")
println("avg Rel onlineNN_gap:  $(mean(RelPrimandOnline_gap))")

println(" ")

println("max onlineDualNN_gap:  $(maximum(DualOnline_gap))")
println("min onlineDualNN_gap:  $(minimum(DualOnline_gap))")
println("avg onlineDualNN_gap:  $(mean(DualOnline_gap))")

println(" ")

println("max Rel onlineDualNN_gap:  $(maximum(RelDualOnline_gap))")
println("min Rel onlineDualNN_gap:  $(minimum(RelDualOnline_gap))")
println("avg Rel onlineDualNN_gap:  $(mean(RelDualOnline_gap))")

println(" ")

println("difference primal variable MAX: $(maximum(primalDiff)) ")
println("difference primal variable MIN: $(minimum(primalDiff)) ")
println("difference primal variable AVG: $(mean(primalDiff)) ")

println(" ")

println("difference first primal variable MAX: $(maximum(primalDiff0)) ")
println("difference first primal variable MIN: $(minimum(primalDiff0)) ")
println("difference first primal variable AVG: $(mean(primalDiff0)) ")

println(" ")
# 
println("difference Optimal primal variable MAX: $(maximum(primalDiffOrigSol)) ")
println("difference optimal primal variable MIN: $(minimum(primalDiffOrigSol)) ")
println("difference optimal primal variable AVG: $(mean(primalDiffOrigSol)) ")
