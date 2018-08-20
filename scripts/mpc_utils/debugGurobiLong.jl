# code is used to debug Gurobi solution for longitudinal contorl

include("GPSKinMPCPathFollowerFrenetLinLongGurobi.jl")
# import GPSKinMPCPathFollowerFrenetLinLongGurobi
# const GurobiModule = GPSKinMPCPathFollowerFrenetLinLongGurobi  # remaining

##########################################
# parameters to be filled in
s_0 = 1.7175185731881666
v_0 = 4.136002952221563
u_0 = 1.999999999999614		# u_{-1}

#optimal solution from gurobi

 
# reference paths
s_ref =  [2.56111 3.46533 4.43053 5.44761 6.52753 7.67051 8.86899 10.1205]'		# add from 1, ..., N
v_ref = [4.35357 4.65301 4.9252 5.22933 5.52616 5.82432 6.10277 6.37296]'		# from 1, ... , N

# solution from gurobi solver
# forgot to multiply H with 2 !
s_gurobi = [2.54472 3.45192 4.43912 5.50632 6.65352 7.88072 9.18792 10.5751]'
v_gurobi = [4.536 4.936 5.336 5.736 6.136 6.536 6.936 7.336]'
dU_gurobi = [-1.34154e-12 7.4335e-13 -1.61721e-12 -6.04961e-13 -6.48093e-13 -2.53736e-12 2.42084e-12 -4.78537e-11]'
u_gurobi = [2.0 2.0 2.0 2.0 2.0 2.0 2.0 2.0]'

# solution from IPOPT
s_ipopt = [2.54472 3.43992 4.39791 5.41471 6.48748 7.61427 8.79378 10.0252]'
v_ipopt =  [4.476 4.78997 5.08396 5.36385 5.63397 5.89755 6.15712 6.415]'
u_ipopt = [1.7 1.56984 1.46993 1.39945 1.35063 1.3179 1.29783 1.28942]'


########################################## 
# system parameters
x_0 = [s_0 ; v_0]
N = 8
dt = 0.2
nx = 2
nu = 1
A = [	1 	dt 		# can be made more exact using matrix exponential
		0	1 	]
B = [ 	0
		dt 		]
g = zeros(nx)

##########################################
#### SYSTEM DYNAMICS CHECK ####
#### x_{k+1} = A*x_k + B*u_k + g ####
X_pred_gurobi = zeros(nx,N+1)
X_pred_gurobi[:,1] = x_0
X_pred_ipopt = zeros(nx,N+1)
X_pred_ipopt[:,1] = x_0
for i = 2 : N+1 # fill in x1, x2, ..., xN
	X_pred_gurobi[:,i] = A*X_pred_gurobi[:,i-1] + B*u_gurobi[i-1] + g
	X_pred_ipopt[:,i] = A*X_pred_ipopt[:,i-1] + B*u_ipopt[i-1] + g
end

s_pred_gurobi = X_pred_gurobi[1,2:N+1]
v_pred_gurobi = X_pred_gurobi[2,2:N+1]
s_pred_ipopt = X_pred_ipopt[1,2:N+1]
v_pred_ipopt = X_pred_ipopt[2,2:N+1]


if norm(s_pred_gurobi-s_gurobi,Inf) > 1e-4
	println("s_gurobi: $(s_gurobi)")
	println("s_predic: $(s_pred)")
elseif norm(v_pred_gurobi - v_gurobi,Inf) > 1e-4
	println("v_gurobi: $(v_gurobi)")
	println("v_predic: $(v_gurobi)")
elseif norm(s_pred_ipopt - s_ipopt,Inf	) > 1e-4
	println("s_ipopt: $(s_ipopt)")
	println("s_predic: $(s_pred_ipopt)")
elseif norm(v_pred_ipopt - v_ipopt,Inf) > 1e-4
	println("v_ipopt: $(v_ipopt)")
	println("v_predic: $(v_pred_ipopt)")
else
	println("=== System Dynamics GUROBI and IPOPT: OK ===")
end



################# Given solutoin of Gurobi/IPOPT, rebuild z-vector #################
n_uxu = 4
z_gurobi = zeros(N*n_uxu)
z_ipopt = zeros(N*n_uxu)
for i = 1 : N
	z_gurobi[(i-1)*n_uxu+1:i*n_uxu] = [ dU_gurobi[i] ; s_gurobi[i] ; v_gurobi[i] ; u_gurobi[i] ]
	z_ipopt[(i-1)*n_uxu+1:i*n_uxu] = [ dU_ipopt[i] ; s_ipopt[i] ; v_ipopt[i] ; u_ipopt[i] ]
end

################# Check if lb <= z_ipopt / z_gurobi <= ub #################
if minimum(z_gurobi - squeeze(GPSKinMPCPathFollowerFrenetLinLongGurobi.lb_gurobi,2)	) < 0	# should be >=0
	println("z_gurobi infeasible 1")
elseif  minimum(GPSKinMPCPathFollowerFrenetLinLongGurobi.ub_gurobi - z_gurobi)	< 0	# should be >=0
	println("z_gurobi infeasible 2")
elseif minimum(z_ipopt - squeeze(GPSKinMPCPathFollowerFrenetLinLongGurobi.lb_gurobi,2)	) < 0	# should be >=0
	println("z_ipopt infeasible 1")
elseif minimum(GPSKinMPCPathFollowerFrenetLinLongGurobi.ub_gurobi - z_ipopt) < 0	# should be >=0
	println("z_ipopt infeasible 2")
else
	println("=== z_ipopt and z_gurobi satisfy inequality constraints ===")
end

################# Check equality (dynamic) constraints of z_ipopt and z_gurobi #################
nx_tilde = 3
beq_gurobi_updated = repmat(GPSKinMPCPathFollowerFrenetLinLongGurobi.g_tilde,N,1)
beq_gurobi_updated[1:nx_tilde] = beq_gurobi_updated[1:nx_tilde] + GPSKinMPCPathFollowerFrenetLinLongGurobi.A_tilde*[x_0 ; u_0] 	# PARAMETER: depends on x0


eqRes_gurobi = GPSKinMPCPathFollowerFrenetLinLongGurobi.Aeq_gurobi*z_gurobi - squeeze(beq_gurobi_updated,2)
eqRes_ipopt = GPSKinMPCPathFollowerFrenetLinLongGurobi.Aeq_gurobi*z_ipopt - squeeze(beq_gurobi_updated,2)

if norm(eqRes_gurobi,Inf) > 1e-4
	println("equality constraint not satisfied for gurobi solution")
elseif norm(eqRes_ipopt,Inf) > 1e-4
	println("equality constraint not satisfied for ipopt soluiton")
else
	println("=== z_ipopt and z_gurobi satisfy equality constraint ===")
end


##################################
#### Check objective Value ####
C_s = 20			# track progress
C_v = 10			# ref velocity tracking weight			
C_acc = 0
C_dacc = 10			# jerk penalty

# compute dU_ipopt
dU_ipopt = zeros(N)
dU_ipopt[1] = u_ipopt[1]-u_0
for i = 2 : N
	dU_ipopt[i] = u_ipopt[i] - u_ipopt[i-1]
end

obj_gurobi = 0
obj_ipopt = 0

for i = 1 : N
	obj_gurobi = obj_gurobi + C_s*(s_gurobi[i]-s_ref[i])^2 + C_v*(v_gurobi[i]-v_ref[i])^2 + C_dacc*(dU_gurobi[i])^2
	obj_ipopt = obj_ipopt + C_s*(s_ipopt[i]-s_ref[i])^2 + C_v*(v_ipopt[i]-v_ref[i])^2 + C_dacc*(dU_ipopt[i])^2
end

if norm(obj_gurobi-obj_ipopt) / norm(obj_ipopt) > 1e-5
	println("!!! gurobi and ipopt solutions don't have same objective value !!!")
	println("obj_gurobi: $(obj_gurobi)")
	println("obj_ipopt: $(obj_ipopt)")
else
	println("=== gurobi and ipopt solutions have same objective values ===")
end



### recompute objective value using z' * H * z + f'*z + const
# determine f which depends on ref values
# update reference trajectories
x_ref = zeros(N*nx,1)
for i = 1 : N  # OLD: i+1!!!!!!!!!!!!!!!!!!!!!!!!
	x_ref[(i-1)*nx+1] = s_ref[i]		# set x_ref, s_ref/v_ref is of dim N+1
	x_ref[(i-1)*nx+2] = v_ref[i]		# set v_ref
end
x_tilde_ref = zeros(N*(nx+nu))
for i = 1 : N
	x_tilde_ref[(i-1)*(nx+nu)+1 : (i-1)*(nx+nu)+nx] = x_ref[(i-1)*nx+1 : (i-1)*nx+nx]
	x_tilde_ref[(i-1)*(nx+nu)+nx+1 : (i-1)*(nx+nu)+nx+nu] = GPSKinMPCPathFollowerFrenetLinLongGurobi.u_ref_init[i]	# u_ref_init always 0, but no no weights
end
u_tilde_ref = zeros(N*nu)	# want to minimize u_tilde = deltaU

z_gurobi_ref = zeros(N*n_uxu) 	# reference point for z_gurobi ; PARAMETER!
for i = 1 : N
	z_gurobi_ref[(i-1)*n_uxu+1 : (i-1)*n_uxu+nu] = u_tilde_ref[(i-1)*nu+1 : i*nu] 		# should be zero for this application
	z_gurobi_ref[(i-1)*n_uxu+nu+1 : i*n_uxu] = x_tilde_ref[(i-1)*(nx+nu)+1 : i*(nu+nx)]
end 
f_gurobi = -2*GPSKinMPCPathFollowerFrenetLinLongGurobi.H_gurobi*z_gurobi_ref


# determine const that depends on ref values
c = z_gurobi_ref'*GPSKinMPCPathFollowerFrenetLinLongGurobi.H_gurobi*z_gurobi_ref

# compute the objective values
objZ_gurobi = z_gurobi'*GPSKinMPCPathFollowerFrenetLinLongGurobi.H_gurobi*z_gurobi+ f_gurobi'*z_gurobi + c
objZ_ipopt = z_ipopt'*GPSKinMPCPathFollowerFrenetLinLongGurobi.H_gurobi*z_ipopt + f_gurobi'*z_ipopt + c


if norm(objZ_gurobi - obj_gurobi)/norm(obj_gurobi) > 1e-5
	println("problem in H, f matrices 1")
	println("objZ_gurobi: $(objZ_gurobi)")
	println("obj_gurobi: $(obj_gurobi)")
elseif norm(objZ_ipopt- obj_ipopt)/norm(obj_ipopt) > 1e-5
	println("problem in H, f matrices 2")
	println("objZ_ipopt: $(objZ_ipopt)")
	println("obj_ipopt: $(obj_ipopt)")
else
	println("=== matrices (H,f) seem correct ===")
end


### Seems the cost matrices (H,f) of z'*H*z + f'*z + c correspond to \sum_k (s_k - s_k^r)^2.....
# try to call solver now
println("##### Resolving Problem with CORRECT H matrices ####")
s_ref_aug = [s_0 ; squeeze(s_ref,2)]
v_ref_aug = [v_0 ; squeeze(v_ref,2)]
u0_gurobi_recomp, solv_time_gurobi, obj_gurobi_recomp, z_gurobi_recomp, dU_gurobi_recomp, s_gurobi_recomp, v_gurobi_recomp, u_gurobi_recomp = GPSKinMPCPathFollowerFrenetLinLongGurobi.solve_gurobi(s_0, v_0, u_0, s_ref_aug, v_ref_aug)
println("--- finished calling Gurobi ---")

println("obj_gurobi_recomp returned from gurobi: $(obj_gurobi_recomp)")

# obj_gurobi_recomp1 = obj_gurobi_recomp + c 	# need to add const term to objective value
# println("obj_gurobi_recomp 1 (incl const): $(obj_gurobi_recomp1)")

obj_gurobi_recomp2 = z_gurobi_recomp'*GPSKinMPCPathFollowerFrenetLinLongGurobi.H_gurobi*z_gurobi_recomp+ f_gurobi'*z_gurobi_recomp 
println("obj_gurobi_recomp 2 (re-evaluated): $(obj_gurobi_recomp2)")
# println("obj_gurobi_recomp (incl const): $(obj_gurobi_recomp+c)")
rel_diff = ( norm(obj_gurobi_recomp-obj_gurobi_recomp2)/obj_gurobi_recomp )
println("relative difference in obj value: $(rel_diff)")


println("old gurobi cost w/o offset (with wrong H-matrix): $(objZ_gurobi-c)")
println("old ipopt cost w/o offset: $(obj_ipopt-c)")

# check feasibility of solution
eqRes_gurobi_recomp = GPSKinMPCPathFollowerFrenetLinLongGurobi.Aeq_gurobi*z_gurobi_recomp - squeeze(beq_gurobi_updated,2)
if norm(eqRes_gurobi_recomp,Inf) > 1e-5
	println("new solution doesn't satisfy equality constraint")
else
	println("new gurobi solution satisfies equality constraint")
end

if minimum(z_gurobi_recomp - squeeze(GPSKinMPCPathFollowerFrenetLinLongGurobi.lb_gurobi,2)	) < 0	# should be >=0
	println("z_gurobi infeasible 1")
elseif  minimum(GPSKinMPCPathFollowerFrenetLinLongGurobi.ub_gurobi - z_gurobi_recomp)	< 0	# should be >=0
	println("z_gurobi infeasible 2")
else
	println("new gurobi solution satisfies inquality constraints")
end
