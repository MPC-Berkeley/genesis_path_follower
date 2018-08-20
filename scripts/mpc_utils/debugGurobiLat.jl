# code is used to debug Gurobi solution for longitudinal contorl

include("GPSKinMPCPathFollowerFrenetLinLatGurobi.jl")
# import GPSKinMPCPathFollowerFrenetLinLongGurobi
# const GurobiModule = GPSKinMPCPathFollowerFrenetLinLongGurobi  # remaining

##########################################
# parameters to be filled in
# ey_0 = 1.0944275703501505
# epsi_0 = -0.0551182313880374
# u_0 = -0.19901144670408796 	# u_prev

# batch two with better data
ey_0 = 0.9548480751243446
epsi_0 = -0.1000209225142008
u_0 = -0.10894442303785415 	# u_prev

# reference paths
# s_pred = [344.72, 345.872, 347.016, 348.154, 349.291, 350.43, 351.576, 352.733, 353.902] 		# add from s0,...,s_N
# v_pred = [5.76197, 5.71904, 5.69058, 5.6824, 5.69626, 5.73097, 5.7828, 5.84611, 5.91445]		# from 1, ... , N

# batch two
s_pred = [348.855, 349.996, 351.146, 352.311, 353.497, 354.709, 355.954, 357.236, 358.557] 		# add from s0,...,s_N
v_pred = [5.7044, 5.75239, 5.82506, 5.92797, 6.06242, 6.22532, 6.40941, 6.60565, 6.80629]		# from 1, ... , N

# k_coeff = [-8.47563e-5, 0.0885254, -30.8124, 3573.87]
# batch 2
k_coeff = [0.00006075262059252357 , -0.06481836196390590443, 23.05374064419753921129 , -2733.36409779475616232958]

# solution from gurobi solver
# ey_gurobi = [1.09443, 0.891604, 0.706934, 0.537889, 0.378816, 0.221394, 0.055384, -0.130336, -0.346339] # includes ey_0
# epsi_gurobi = [-0.0551182, -0.0473849, -0.0434027, -0.0434067, -0.048425, -0.0596165, -0.07774, -0.102691, -0.133085] # includes epsi_0
# u_gurobi = [-0.197772, -0.186619, -0.171994, -0.157983, -0.146843, -0.139424, -0.135529, -0.134237]

# 2nd batch
ey_gurobi = [0.954848, 0.768179, 0.600342, 0.445372, 0.299137, 0.158365, 0.0200547, -0.118899, -0.262062] # includes ey_0
epsi_gurobi = [-0.100021, -0.0898338, -0.0858602, -0.0843606, -0.0834595, -0.0825712, -0.0819882, -0.0826244, -0.0859127] # includes epsi_0
u_gurobi = [-0.104049, -0.0917017, -0.0771561, -0.0637777, -0.0534039, -0.0466534, -0.0432086, -0.0421107]


# solution from IPOPT
# ey_ipopt = [1.09443, 0.892675, 0.709147, 0.541104, 0.382771, 0.225767, 0.0598248, -0.126189, -0.342857] # includes ey_0
# epsi_ipopt =  [-0.0551182, -0.0467703, -0.0425357, -0.0425312, -0.0476958, -0.0591243, -0.0775326, -0.10279, -0.133499] # includes epsi_0
# u_ipopt = [-0.196252, -0.18599, -0.171973, -0.15835, -0.147436, -0.140132, -0.136285, -0.135005]

# 2nd batch
ey_ipopt = [0.954848, 0.772383, 0.609065, 0.458095, 0.314811, 0.175609, 0.0372615, -0.103504, -0.250408] # includes ey_0
epsi_ipopt = [-0.100021, -0.0874202, -0.0824468, -0.0809336, -0.0806706, -0.0808231, -0.0815105, -0.083538, -0.0882815]  # includes epsi_0
u_ipopt = [-0.0980199, -0.0892249, -0.0771229, -0.0653114, -0.0558504, -0.0495615, -0.0463018, -0.0452498]


#### SYSTEM DYNAMICS CHECK ####
#### x_{k+1} = A*x_k + B*u_k + g ####

x_0 = [ey_0 ; epsi_0]
N = 8
dt = 0.2
nx = 2
nu = 1

L_a = GPSKinMPCPathFollowerFrenetLinLatGurobi.L_a
L_b = GPSKinMPCPathFollowerFrenetLinLatGurobi.L_b

# construct system matrices
A = zeros(nx, nx, N)
B = zeros(nx, nu, N)
g = zeros(nx, N)
for i = 1 : N
	A[:,:,i] = [	1	dt*v_pred[i] 
					0		1			]
	B[:,:,i] = [	dt*v_pred[i]*L_b/(L_a+L_b) 
					dt*v_pred[i] / (L_a + L_b)	]
	g[:,i] = [ 	0	# column vector
				-dt*v_pred[i]*(k_coeff[1]*s_pred[i]^3 + k_coeff[2]*s_pred[i]^2 + k_coeff[3]*s_pred[i] + k_coeff[4]) 	]
end

X_pred_gurobi = zeros(nx,N+1)
X_pred_gurobi[:,1] = x_0
X_pred_ipopt = zeros(nx,N+1)
X_pred_ipopt[:,1] = x_0

for i = 2 : N+1 # fill in x1, x2, ..., xN
	X_pred_gurobi[:,i] = A[:,:,i-1]*X_pred_gurobi[:,i-1] + B[:,:,i-1]*u_gurobi[i-1] + g[:,i-1]
	X_pred_ipopt[:,i] = A[:,:,i-1]*X_pred_ipopt[:,i-1] + B[:,:,i-1]*u_ipopt[i-1] + g[:,i-1]
end

ey_pred_gurobi = X_pred_gurobi[1,1:N+1]
epsi_pred_gurobi = X_pred_gurobi[2,1:N+1]
ey_pred_ipopt = X_pred_ipopt[1,1:N+1]
epsi_pred_ipopt = X_pred_ipopt[2,1:N+1]

# epsi_1 is pretty large
if norm(ey_pred_gurobi-ey_gurobi,Inf) > 1e-4 	# 0.0008098428022678128
	println("ey_gurobi: $(ey_gurobi)")
	println("ey_predic: $(ey_pred_gurobi)")
elseif norm(epsi_pred_gurobi - epsi_gurobi,Inf) > 1e-4 # 0.0001948524614348579
	println("epsi_gurobi: $(epsi_gurobi)")
	println("epsi_predic: $(epsi_pred_gurobi)")
elseif norm(ey_pred_ipopt - ey_ipopt,Inf	) > 1e-4  # 0.0008086712242413308
	println("ey_ipopt: $(ey_ipopt)")
	println("ey_predic: $(ey_pred_ipopt)")
elseif norm(epsi_pred_ipopt - epsi_ipopt,Inf) > 1e-4  # 0.00019478001175067539
	println("epsi_ipopt: $(epsi_ipopt)")
	println("epsi_predic: $(epsi_pred_ipopt)")
else
	println("=== System Dynamics GUROBI and IPOPT: OK ===")
end


# taken from ipopt implementation
ey_ipopt2 = zeros(N+1)
ey_ipopt2[1] = ey_0
epsi_ipopt2 = zeros(N+1)
epsi_ipopt2[1] = epsi_0

ey_gurobi2 = zeros(N+1)
ey_gurobi2[1] = ey_0
epsi_gurobi2 = zeros(N+1)
epsi_gurobi2[1] = epsi_0

K_r = zeros(N)
bta_approx = zeros(N)
bta_approx_gurobi = zeros(N)

for i in 1:N
	K_r[i] = k_coeff[1]*s_pred[i]^3 + k_coeff[2]*s_pred[i]^2 + k_coeff[3]*s_pred[i] + k_coeff[4]   # Predicted Curvature; Points of Linearization
	bta_approx[i] = L_b / (L_a + L_b) * u_ipopt[i] 
	bta_approx_gurobi[i] = L_b / (L_a + L_b) * u_gurobi[i] 

	ey_ipopt2[i+1] = ey_ipopt2[i] + dt*( v_pred[i]*(epsi_ipopt2[i] + bta_approx[i])) 
   	epsi_ipopt2[i+1]  = epsi_ipopt2[i]    + dt*( v_pred[i]/L_b*bta_approx[i] - v_pred[i]*K_r[i] )
	
	# "restart" each time step with ey_ipopt / epsi_ipopt
	# ey_ipopt2[i+1] = ey_ipopt[i] + dt*( v_pred[i]*(epsi_ipopt[i] + bta_approx[i])) 
   	# epsi_ipopt2[i+1]  = epsi_ipopt[i]    + dt*( v_pred[i]/L_b*bta_approx[i] - v_pred[i]*K_r[i] )

   	ey_gurobi2[i+1] = ey_gurobi2[i] + dt*( v_pred[i]*(epsi_gurobi2[i] + bta_approx_gurobi[i])) 
   	epsi_gurobi2[i+1]  = epsi_gurobi2[i]    + dt*( v_pred[i]/L_b*bta_approx_gurobi[i] - v_pred[i]*K_r[i] )

   	# "restart" each time step with ey_gurobi / epsi_gurobi
   	# ey_gurobi2[i+1] = ey_gurobi[i] + dt*( v_pred[i]*(epsi_gurobi[i] + bta_approx_gurobi[i])) # use solution vector to re-propagate
   	# epsi_gurobi2[i+1]  = epsi_gurobi[i]    + dt*( v_pred[i]/L_b*bta_approx_gurobi[i] - v_pred[i]*K_r[i] )
end

norm(ey_ipopt2 - ey_pred_ipopt,Inf)		# tiny --> (A,B,g) seem correct
norm(epsi_ipopt2 - epsi_pred_ipopt,Inf)	# tiny --> (A,B,g) seem correct
# norm(ey_ipopt2 - ey_ipopt,Inf)			# 0.05021048526331179; 2.429649849533533e-6 if "restarted" with ey_ipopt
# norm(epsi_ipopt2 - epsi_ipopt,Inf)		# 0.0124546139953301; 0.0015774704744410117 if "restarted" with ey_ipopt
# 2nd batch
norm(ey_ipopt2 - ey_ipopt,Inf)			# 0.05021048526331179; 7.808783477891712e-7 if "restarted" with ey_ipopt
norm(epsi_ipopt2 - epsi_ipopt,Inf)		# 0.0124546139953301; 0.00018890853543568342 if "restarted" with ey_ipopt
 # conjecture: the large values in K_r screw things up and propage errors

norm(ey_gurobi2 - ey_pred_gurobi,Inf)		# tiny --> (A,B,g) seem correct
norm(epsi_gurobi2 - epsi_pred_gurobi,Inf)	# tiny --> (A,B,g) seem correct
# norm(ey_gurobi2 - ey_gurobi,Inf)		# 0.0502090760352249; 2.429649849533533e-6 if "restarted" with ey_gurobi
# norm(epsi_gurobi2 - epsi_gurobi,Inf)	# 0.01245405355462842; 0.0015774806077743508 if "restarted" with epsi_gurobi
norm(ey_gurobi2 - ey_gurobi,Inf)		# 0.0502090760352249; 8.340549827179355e-7 if "restarted" with ey_gurobi
norm(epsi_gurobi2 - epsi_gurobi,Inf)	# 0.01245405355462842; 0.00018900691859358165 if "restarted" with epsi_gurobi

# # test - overwrite with computed solution
# ey_gurobi2 = ey_pred_gurobi
# epsi_gurobi2 = epsi_pred_gurobi

# ey_ipopt2 = ey_pred_ipopt
# epsi_ipopt2 = epsi_pred_ipopt


# ################# Given solution of Gurobi/IPOPT, rebuild z-vector #################

# compute dU_ipopt
dU_ipopt = zeros(N)
dU_gurobi = zeros(N)
dU_ipopt[1] = u_ipopt[1]-u_0
dU_gurobi[1] = u_gurobi[1]-u_0
for i = 2 : N
	dU_ipopt[i] = u_ipopt[i] - u_ipopt[i-1]
	dU_gurobi[i] = u_gurobi[i] - u_gurobi[i-1]
end


n_uxu = 4
z_gurobi = zeros(N*n_uxu)
z_gurobi2 = zeros(N*n_uxu)
z_ipopt = zeros(N*n_uxu)
z_ipopt2 = zeros(N*n_uxu)
for i = 1 : N
	# maybe also try pred_gurobi and pred_ipopt!!!!!!!!!!!!!!!!!
	# note that: ey_gurobi starts at ey_0! same for epsi_gurobi
	z_gurobi[(i-1)*n_uxu+1:i*n_uxu] = [ dU_gurobi[i] ; ey_gurobi[i+1] ; epsi_gurobi[i+1] ; u_gurobi[i] ]
	z_gurobi2[(i-1)*n_uxu+1:i*n_uxu] = [ dU_gurobi[i] ; ey_gurobi2[i+1] ; epsi_gurobi2[i+1] ; u_gurobi[i] ]
	z_ipopt[(i-1)*n_uxu+1:i*n_uxu] = [ dU_ipopt[i] ; ey_ipopt[i+1] ; epsi_ipopt[i+1] ; u_ipopt[i] ]
	z_ipopt2[(i-1)*n_uxu+1:i*n_uxu] = [ dU_ipopt[i] ; ey_ipopt2[i+1] ; epsi_ipopt2[i+1] ; u_ipopt[i] ]
end

################# Check if lb <= z_ipopt / z_gurobi <= ub #################
if minimum(z_gurobi - GPSKinMPCPathFollowerFrenetLinLatGurobi.lb_gurobi	) < 0	# should be >=0
	println("z_gurobi infeasible 1")
elseif  minimum(GPSKinMPCPathFollowerFrenetLinLatGurobi.ub_gurobi - z_gurobi )	< 0	# should be >=0
	println("z_gurobi infeasible 2")
elseif minimum(z_ipopt - GPSKinMPCPathFollowerFrenetLinLatGurobi.lb_gurobi ) < 0	# should be >=0
	println("z_ipopt infeasible 1")
elseif minimum(GPSKinMPCPathFollowerFrenetLinLatGurobi.ub_gurobi - z_ipopt) < 0	# should be >=0
	println("z_ipopt infeasible 2")
else
	println("=== z_ipopt and z_gurobi satisfy inequality constraints ===")
end

################# Check equality (dynamic) constraints of z_ipopt and z_gurobi #################
nx_tilde = 3
nu_tilde = 1

# x_tilde transformation
# update system matrices for tilde-notation
A_tilde = zeros(nx+nu,nx+nu,N)
B_tilde = zeros(nx+nu,nu,N)
g_tilde = zeros(nx+nu,N)
for i = 1 : N 
	A_tilde[:,:,i] = [ 	A[:,:,i]  		B[:,:,i] 
						zeros(nu,nx)   			eye(nu)			]
	B_tilde[:,:,i] = [	B[:,:,i] 	;  	eye(nu)	]
	g_tilde[:,i] =   [	g[:,i]		; 	zeros(nu) ]
end

# z-transformation
Aeq_gurobi = zeros(N*nx_tilde , N*(nx_tilde+nu_tilde))
Aeq_gurobi[1:nx_tilde, 1:(nx_tilde+nu_tilde)] = [-B_tilde[:,:,1] eye(nx_tilde)] 	# fill out first row associated with x_tilde_1
for i = 2 : N  	# fill out rows associated to x_tilde_2, ... , x_tilde_N
	Aeq_gurobi[ (i-1)*nx_tilde+1 : i*nx_tilde  , (i-2)*(nu_tilde+nx_tilde)+(nu_tilde)+1 : (i-2)*(nu_tilde+nx_tilde)+nu_tilde+(nx_tilde+nu_tilde+nx_tilde)    ] = [-A_tilde[:,:,i] -B_tilde[:,:,i] eye(nx_tilde)]
end

# right-hand-size of equality constraint
beq_gurobi = zeros(N*nx_tilde)
for i = 1 : N
	beq_gurobi[(i-1)*nx_tilde+1:i*nx_tilde] = g_tilde[:,i]
end
beq_gurobi[1:nx_tilde] = beq_gurobi[1:nx_tilde] + A_tilde[:,:,1]*[x_0 ; u_0] 	# PARAMETER: depends on x0

# test the original gurobi/ipopt solutions
eqRes_gurobi = Aeq_gurobi*z_gurobi - beq_gurobi      # 2nd element large, everything small
eqRes_gurobi2 = Aeq_gurobi*z_gurobi2 - beq_gurobi    
eqRes_ipopt = Aeq_gurobi*z_ipopt - beq_gurobi
eqRes_ipopt2 = Aeq_gurobi*z_ipopt2 - beq_gurobi 	 
	
if norm(eqRes_gurobi,Inf) > 1e-4 	# 0.0015775119958117492 (due to k_coeff?????); 0.00018908440439276142 with better k_coeff
	println("equality constraint not satisfied for gurobi solution")
elseif norm(eqRes_ipopt,Inf) > 1e-4 # 0.001577501862478417 (due to k_coeff????); 0.00018898602123487707 on 2nd trial with better k_coeff
	println("equality constraint not satisfied for ipopt soluiton")
else
	println("=== z_ipopt and z_gurobi satisfy equality constraint ===")
end

norm(eqRes_gurobi2, Inf) # =1.1102230246251565e-16 (same with ey_pred_gurobi): -> modelling correct
# = 0.0018211371661522313 (0.00021738730143316953 2nd batch) if "restarted" --> K_coeff not accurate enough
norm(eqRes_ipopt2, Inf)  # =3.122502256758253e-17 (same with ey_pred_ipopt): -> modelling Aeq,beq correct
# = 0.0018223588275759761 (0.00021738730143316953 2nd batch) if "restarted" --> K_coeff not accurate enough


# ##################################
# #### Check objective Value ####
C_ey = 5.0				# lateral deviation
C_epsi = 1.0			# heading deviation
C_ddf	 = 1000	# 3e4			# derivative of tire angle input
C_df	 = 0.0	# 150			# tire angle input

obj_gurobi = 0
obj_ipopt = 0

for i = 1 : N
	obj_gurobi = obj_gurobi + C_ey*(ey_gurobi[i+1])^2 + C_epsi*(epsi_gurobi[i+1])^2 + C_ddf*(dU_gurobi[i])^2
	obj_ipopt = obj_ipopt + C_ey*(ey_ipopt[i+1])^2 + C_epsi*(epsi_ipopt[i+1])^2 + C_ddf*(dU_ipopt[i])^2
end

if norm(obj_gurobi-obj_ipopt) / norm(obj_ipopt) > 1e-5
	println("!!! gurobi and ipopt solutions don't have same objective value !!!")
	println("obj_gurobi: $(obj_gurobi)")
	println("obj_ipopt: $(obj_ipopt)")
else
	println("=== gurobi and ipopt solutions have same objective values ===")
end

# ### recompute objective value using z' * H * z + f'*z + const
# notice that f = 0; since all ref_values = 0 ==> no const either
# # compute the objective values
objZ_gurobi = z_gurobi'*GPSKinMPCPathFollowerFrenetLinLatGurobi.H_gurobi*z_gurobi
objZ_ipopt = z_ipopt'*GPSKinMPCPathFollowerFrenetLinLatGurobi.H_gurobi*z_ipopt 

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

# ###############################################################################
# ###############################################################################
# ###############################################################################





# ### Seems the cost matrices (H,f) of z'*H*z + f'*z + c correspond to \sum_k (s_k - s_k^r)^2.....
# # try to call solver now
# println("##### Resolving Problem with CORRECT H matrices ####")
# s_ref_aug = [s_0 ; squeeze(s_ref,2)]
# v_ref_aug = [v_0 ; squeeze(v_ref,2)]
# u0_gurobi_recomp, solv_time_gurobi, obj_gurobi_recomp, z_gurobi_recomp, dU_gurobi_recomp, s_gurobi_recomp, v_gurobi_recomp, u_gurobi_recomp = GPSKinMPCPathFollowerFrenetLinLongGurobi.solve_gurobi(s_0, v_0, u_0, s_ref_aug, v_ref_aug)
# println("--- finished calling Gurobi ---")

# println("obj_gurobi_recomp returned from gurobi: $(obj_gurobi_recomp)")

# # obj_gurobi_recomp1 = obj_gurobi_recomp + c 	# need to add const term to objective value
# # println("obj_gurobi_recomp 1 (incl const): $(obj_gurobi_recomp1)")

# obj_gurobi_recomp2 = z_gurobi_recomp'*GPSKinMPCPathFollowerFrenetLinLongGurobi.H_gurobi*z_gurobi_recomp+ f_gurobi'*z_gurobi_recomp 
# println("obj_gurobi_recomp 2 (re-evaluated): $(obj_gurobi_recomp2)")
# # println("obj_gurobi_recomp (incl const): $(obj_gurobi_recomp+c)")
# rel_diff = ( norm(obj_gurobi_recomp-obj_gurobi_recomp2)/obj_gurobi_recomp )
# println("relative difference in obj value: $(rel_diff)")


# println("old gurobi cost w/o offset (with wrong H-matrix): $(objZ_gurobi-c)")
# println("old ipopt cost w/o offset: $(obj_ipopt-c)")

# # check feasibility of solution
# eqRes_gurobi_recomp = GPSKinMPCPathFollowerFrenetLinLongGurobi.Aeq_gurobi*z_gurobi_recomp - squeeze(beq_gurobi_updated,2)
# if norm(eqRes_gurobi_recomp,Inf) > 1e-5
# 	println("new solution doesn't satisfy equality constraint")
# else
# 	println("new gurobi solution satisfies equality constraint")
# end

# if minimum(z_gurobi_recomp - squeeze(GPSKinMPCPathFollowerFrenetLinLongGurobi.lb_gurobi,2)	) < 0	# should be >=0
# 	println("z_gurobi infeasible 1")
# elseif  minimum(GPSKinMPCPathFollowerFrenetLinLongGurobi.ub_gurobi - z_gurobi_recomp)	< 0	# should be >=0
# 	println("z_gurobi infeasible 2")
# else
# 	println("new gurobi solution satisfies inquality constraints")
# end
