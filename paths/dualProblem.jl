# Julia Trial Code for estimating cost of Dual Problem 

using JuMP
using Gurobi 

module dualCostLong
	__precompile__()

	# Defining the simple system 

	N = 3 
	nx = 2
	nw = 2
	nu = 1
	dt = 0.20


	A = [1 dt
		 0 1]

	B = [0
		 dt]
    
    E = [1 0
    	 0 1]

    w = zeros(nx)				# calling the g w here because I get confused


    # define cost functions
    C_s = 20			# track progress
	C_v = 10;			# ref velocity tracking weight			
	C_acc = 0
	C_dacc = 11;		# 20 too high; 10 OK for med speed; 10 a big jerky for high speed; 13 too high

	Q = diagm([C_s ; C_v])	# create diagonal matrix
	R = C_acc
	Rdelta = C_dacc


   # define (box) constraints
	
	largeNumber = 1e5;		# use this number for variables that are not upper/lower bounded
	v_min = 0.0				# vel bounds (m/s)
	v_max = 20.0	
	a_max = 2.0				# acceleration and deceleration bound, m/s^2
	a_dmax = 1.5			# jerk bound, m/s^3

	x_lb = [	-largeNumber	# make sure car doesnt travel more than largeNumber [m]
				v_min		]
	x_ub = [	largeNumber
				v_max		]

	u_lb = -a_max
	u_ub = a_max

	dU_lb = -a_dmax*dt 	    # double check if *dt is needed (or not)
	dU_ub = a_dmax*dt

    # build references (should be passed on as arguments later on)
	
	# get init x_ref for one initial solve (to speed up Julia)
    s_ref_init   = 1.0*collect(dt:dt:(N)*dt)		# target s(1), ..., s(N)
	v_ref_init = ones(N,1)						    # reference velocity 
	x_ref_init = zeros(N*nx,1)
	for i = 1 : N
		x_ref_init[(i-1)*nx+1] = s_ref_init[i]		# set x_ref
		x_ref_init[(i-1)*nx+2] = v_ref_init[i]		# set v_ref
	end

	# input reference
	u_ref_init = zeros(N,1)	# if not used, set cost to zeros

	# get Initial state and input
	# should be done dynamically later on
	s0_init = 0
	v0_init = 0
	x0_init = [s0_init ; v0_init]
	u0_init = 0
	
	# ================== Transformation 1 ======================
	# augment state and redefine system dynamics (A,B,g) and constraints
	# x_tilde_k := (x_k , u_{k-1})
	# u_tilde_k := (u_k - u_{k-1})

	A_tilde = [	A 				B
				zeros(nu,nx) 	eye(nu)	]

	B_tilde = [	B ; eye(nu)	]

	w_tilde = [	w ;	zeros(nu) ]
 	E_tilde = blkdiag(E, eye(1))

	# Write these appended matrices along the whole horizon for big QP

	[A_tilde_vec, B_tilde_vec, E_tilde_vec] = appendMat(N, nx, nu, A_tilde, B_tilde, E_tilde)

	w_tilde_vec = repmat(w_tilde,N, 1)  

	#################################################################

	x_tilde_lb = [x_lb ; u_lb]
	x_tilde_ub = [x_ub ; u_ub]
	u_tilde_lb = dU_lb
	u_tilde_ub = dU_ub

	### Now write these constraints in Fx <= f and Gu < = g format

	G_tilde = [eye(nu) ; -eye(nu)]
	g_tilde = [u_tilde_ub; -u_tilde_lb]
	ng = length(g)
	
	# Concatenate input (tilde) constraints
	G_tilde_vec = kron(eye(N), G_tilde)
	g_tilde_vec = repmat(g_tilde,N, 1)

	# Appended State constraints (tilde)
	F_tilde = [eye(nx+nu) ; -eye(nx+nu)]
	f_tilde = [x_tilde_ub ; -x_tilde_lb]
	nf = length(f_tilde);

	# Concatenate appended state (tilde) constraints
	F_tilde_vec = kron(eye(N), F_tilde)
	f_tilde_vec = repmat(f_tilde,N, 1)   
	
	### Cost matrices for appended states 
	Q_tilde = [	Q 			zeros(nx,nu) 		# may also use cat(?)
				zeros(nu,nx)	R 			]	# actually not needed

	R_tilde = Rdelta
	
	## Now stack them along the horizon for big cost matrices 
	Q_tilde_vec = [] 
	R_tilde_vec = []
	
	for i=1:N
		Q_tilde_vec = blkdiag(Q_tilde_vec,Q_tilde)
		R_tilde_vec = blkdiag(R_tilde_vec,R_tilde)
	end
	#########################################################


	x_tilde_0_init = [x0_init ; u0_init]	# ONLY change in the dual formulation (if taken as constraint)

	x_tilde_ref_init = zeros(N*(nx+nu))
	
	for i = 1 : N
		x_tilde_ref_init[(i-1)*(nx+nu)+1 : (i-1)*(nx+nu)+nx] = x_ref_init[(i-1)*nx+1 : i*nx]
		x_tilde_ref_init[(i-1)*(nx+nu)+nx+1 : (i-1)*(nx+nu)+nx+nu] = u_ref_init[i]
	end

	u_tilde_ref_init = zeros(N*nu) 			# goal is to minimize uTilde = (acc_k - acc_{k-1})

	# All things are now similar to Chassis problem. Dual formulation should 
	# be standard. Just need to change the matrices accordingly and add state
	# constraints. 

	# Dual cost computation 

     Q_dual = 2*(B_tilde_vec'*Q_tilde_vec*B_tilde_vec + R_tilde_vec);
     
     c_dual = (2*x_tilde_0_init'*A_tilde_vec'*Q_tilde_vec*B_tilde_vec + 2*w_tilde_vec'*B_tilde_vec'*Q_tilde_vec*B_tilde_vec...
              -2*x_tilde_ref_init'*Q_tilde_vec*B_tilde_vec)';
     
     const_dual = x_tilde_0_init'*A_tilde_vec'*Q_tilde_vec*A_tilde_vec*x_tilde_0_init + 2*x_tilde_0_init'*A_tilde_vec'*Q_tilde_vec*E_tilde_vec*w_tilde_vec...
                  + w_tilde_vec'*E_tilde_vec'*Q_tilde_vec*E_tilde_vec*w_tilde_vec...
                  - 2*x_tilde_0_init'*A_tilde_vec'*Q_tilde_vec*x_tilde_ref_init - 2*w_tilde_vec'*E_tilde_vec'*Q_tilde_vec*x_tilde_ref_init...
                  + x_tilde_ref_init'*Q_tilde_vec*x_tilde_ref_init;
        
     C_dual = [F_tilde_vec*B_tilde_vec; G_tilde_vec]		        # Adding state constraints 
     d_dual = [f_tilde_vec - F_tilde_vec*A_tilde_vec*x_tilde_0_init - F_tilde_vec*E_tilde_vec*w_tilde_vec;  g_tilde_vec]
     Qdual_tmp = C_dual*(Q_dual\(C_dual'))
     Qdual_tmp = 0.5*(Qdual_tmp+Qdual_tmp') + 0e-5*eye(N*(nf+ng))
    
    
     obj_Dual = -1/2 * L_test'*Qdual_tmp*L_test - (C_dual*(Q_dual\c_dual)+d_dual)'*L_test - 1/2*c_dual'*(Q_dual\c_dual) + const_dual
 


function [Ax_vec, Bx_vec, Ex_vec] = appendMat(N, nx, nu, nw, A, B, E)

	Ax_vec = zeros(N*nx, nx)
        for ii = 1 : N
            Ax_vec[1+(ii-1)*nx:ii*nx,:] = A^ii
        end

	Bx_vec = zeros(N*nx, nu*N)

        for ii = 0 : N-1
            for jj = 0 : ii-1
                Bx_vec[1+ii*nx:(ii+1)*nx, 1+jj*nu:  (jj+1)*nu] = A^(ii-jj)*B
            end
            Bx_vec[1+ii*nx:(ii+1)*nx, 1+ii*nu:(ii+1)*nu] = B
        end

    Ex_vec = zeros(N*nx, nw*N)

        for ii = 0 : N-1
            for jj = 0 : ii-1
                Ex_vec[1+ii*nx:(ii+1)*nx, 1+jj*nw:  (jj+1)*nw] = A^(ii-jj)*E
            end
            Ex_vec[1+ii*nx:(ii+1)*nx, 1+ii*nw:(ii+1)*nw] = E
        end

end