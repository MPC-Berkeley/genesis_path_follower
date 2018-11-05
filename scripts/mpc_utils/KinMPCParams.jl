#!/usr/bin/env julia

#=
Defines MPC parameters that other files can load from

Author: GXZ
Date: 	25 Oct 2018
=# 

# This is an implementation of MPC using the kinematic bicycle model found here:
# http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf

__precompile__()

module KinMPCParams

	platform = "abby" 		# GXZ's laptop
	# platform = "nuvo"		# Black Genesis

	dt      = 0.2			# model discretization time, td (s)
	# dt 		= 0.1

	N       = 8				# horizon
	# N		= 16

	L_a 	= 1.5213		# from CoG to front axle (according to Jongsang)
	L_b 	= 1.4987		# from CoG to rear axle (according to Jongsang)

	# longitudinal parameters
	nx_long	= 2				# dimension of x = (s,v)
	nu_long = 1				# number of inputs u = a
	C_s 	= 20			# track progress
	C_v 	= 10;			# ref velocity tracking weight			
	C_acc 	= 0
	C_dacc 	= 11;		    # 20 too high; 10 OK for med speed; 10 a big jerky for high speed; 13 too high
	v_min = 0.0				# vel bounds (m/s)
	v_max = 20.0	
	a_max = 2.0				# acceleration and deceleration bound, m/s^2
	a_dmax = 1.5			# jerk bound, m/s^3

	# lateral parameters
	nx_lat 		= 2						# dimension of x = (ey,epsi)
	nu_lat 		= 1						# number of inputs u = df
	C_ey 		= 5.0					# lateral deviation
    C_epsi 		= 0.0
	C_df	 	= 0.0					# 150			# tire angle input
	C_ddf	 	= 1000.0				# 3e4			# derivative of tire angle input
	df_max 		= 0.5					# steering
	ddf_max 	= 0.5					# change in steering

    ## The below are all for random data generation for NN train 
	ey_lb       = -1
	ey_ub		=  1	
	epsi_lb     = -0.1
	epsi_ub		=  0.1					# changed from RFS data  	
	
	curv_lb  	=  -0.1
	curv_ub 	=   0.1			
	vpred_lb   =  0
	vpred_ub   =  12					
	
end
