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
dfprev_lb =  KinMPCParams.dfprev_lb
dfprev_ub =  KinMPCParams.dfprev_ub

v_lb 		= KinMPCParams.vpred_lb 
v_ub 		= KinMPCParams.vpred_ub
dv_lb 		= KinMPCParams.dv_lb
dv_ub 		= KinMPCParams.dv_ub
curv_lb 	= KinMPCParams.curv_lb
curv_ub 	= KinMPCParams.curv_ub
dcurv_ub 	= KinMPCParams.dcurv_ub
dcurv_lb 	= KinMPCParams.dcurv_lb


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
num_DataPoints = 3e4								# Training data count 

# solv_time_all = zeros(num_DataPoints)

inputParam_lat		= zeros(num_DataPoints,3+2*N)
outputParamDual_lat = zeros(num_DataPoints, N*(nf+ng))
# outputParamDf_lat   = zeros(num_DataPoints, N*(nu))
outputParamDdf_lat  = zeros(num_DataPoints, N*(nu))
optVal_lat 			= zeros(num_DataPoints)

dual_gap 	= zeros(num_DataPoints)
Reldual_gap = zeros(num_DataPoints)

# counts number of errors when solving the optimization problems
primStatusError = 0
dualStatusError = 0

iii = 1

while iii <= num_DataPoints
	
	if mod(iii,100) == 0
		println("----------- $(iii) ------------")
	end

	# extract appropriate parameters	
	# RANDOM data extraction
 	ey_0 = ey_lb + (ey_ub-ey_lb)*rand(1)				
 	epsi_0 = epsi_lb + (epsi_ub-epsi_lb)*rand(1) 
 	u_0 = dfprev_lb + (dfprev_ub-dfprev_lb)*rand(1) 	

	# dv_pred = dv_lb + (dv_ub-dv_lb)*rand(1,N-1)
 # 	v_pred = zeros(1,N)
 # 	v_pred[1] = v_lb + (v_ub-v_lb)*rand()
 # 	for kk = 2 : N
 # 		v_pred[kk] = v_pred[kk-1] + dv_pred[kk-1]
 # 	end	
 	v_pred = v_lb + (v_ub-v_lb)*rand(1,N)

	# dcurv_pred = dcurv_lb + (dcurv_ub-dcurv_lb)*rand(1,N-1)
 # 	c_pred = zeros(1,N)
 # 	c_pred[1] = curv_lb + (curv_ub-curv_lb)*rand()
 # 	for kk = 2 : N
 # 		c_pred[kk] = c_pred[kk-1] + dcurv_pred[kk-1]
 # 	end	
 	c_pred = curv_lb + (curv_ub-curv_lb)*rand(1,N)

	# Appending parameters here Geokkhge style
	# vc_pred = v_pred.*c_pred

 	###########################################
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
						-dt*v_pred[i].*c_pred[i] 	]
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

	############ solve Primal
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


	######################## dual stuff now
	Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
     
    c_dual = (2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*B_tilde_vec +
    	      - 2*x_tilde_ref'*Q_tilde_vec*B_tilde_vec)'
     
    const_dual = x_tilde_0'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0 + 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  + g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*g_tilde_vec +
                  - 2*x_tilde_0'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref - 2*g_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref +
                  + x_tilde_ref'*Q_tilde_vec*x_tilde_ref
        
    # C_dual = [F_tilde_vec*B_tilde_vec; Fu_tilde_vec]		        # Adding state constraints 
    C_dual = Fu_tilde_vec		        # Adding state constraints 
    # d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0 - F_tilde_vec*E_tilde_vec*g_tilde_vec;  fu_tilde_vec]
    d_dual = fu_tilde_vec
    Qdual_tmp = C_dual*(Q_dual\(C_dual'))
    Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))

    mdlD = Model(solver=GurobiSolver(Presolve=0, LogToConsole=0))
	@variable(mdlD, L_test[1:N*(nf+ng)])  	# decision variable; contains everything
	@objective(mdlD, Max, -1/2 * L_test'*Qdual_tmp*L_test - (C_dual*(Q_dual\c_dual)+d_dual)'*L_test - 1/2*c_dual'*(Q_dual\c_dual) + const_dual)
	@constraint(mdlD, -L_test .<= 0)

	statusD = solve(mdlD)
 	if !(statusD == :Optimal)
		@goto label1 
	end

  	###### DATA collection now #####	
  	# extract solution
	obj_primal = getobjectivevalue(mdl)
	ddf_pred_opt = getvalue(u_tilde_vec)
	obj_dual = getobjectivevalue(mdlD)
	L_test_opt = getvalue(L_test)

	## store the primal solution too as output gonna change now 
	inputParam_lat[iii,:] = [ey_0 epsi_0 u_0 v_pred c_pred]	
	outputParamDdf_lat[iii,:] = ddf_pred_opt
	outputParamDual_lat[iii,:] = L_test_opt
	optVal_lat[iii] = obj_primal

 	###########################################################	
	dual_gap[iii] = (obj_primal - obj_dual)
	Reldual_gap[iii] = (obj_primal - obj_dual)/obj_primal
 
	iii = iii + 1 

	@label label1
end

println("****************************")
println("$(num_DataPoints) data generated")

println(" ")

println("primal status errors:  $(primStatusError)")
println("dual status errors:  $(dualStatusError)")

println(" ")

println("max dual_gap:  $(maximum(dual_gap))")
println("min dual_gap:  $(minimum(dual_gap))")
println("avg dual_gap:  $(mean(dual_gap))")

println(" ")

println("max Rel dual_gap:  $(minimum(Reldual_gap))")
println("min Rel dual_gap:  $(minimum(Reldual_gap))")
println("avg Rel dual_gap:  $(mean(Reldual_gap))")



### save data
matwrite("NN_test_CPGDay5_BadRandTrainingDataLat30k.mat", Dict(
	"inputParam_lat" => inputParam_lat,
	"outputParamDdf_lat" => outputParamDdf_lat,
	"outputParamDual_lat" => outputParamDual_lat,
	"optVal_lat" => optVal_lat
))

println("---- done extracting and saving solutions for LAT control ----")
