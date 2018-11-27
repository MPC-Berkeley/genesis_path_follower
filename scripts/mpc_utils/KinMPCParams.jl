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



	largeNumber = 1e5;

	dt      = 0.2			# model discretization time, td (s)
	# dt 		= 0.1

	N       = 4			# horizon
	# N		= 16

	L_a 	= 1.5313		# from CoG to front axle (according to Jongsang)
	L_b 	= 1.4987		# from CoG to rear axle (according to Jongsang)

	# longitudinal parameters
	nx_long	= 2				# dimension of x = (s,v)
	nu_long = 1				# number of inputs u = a
	C_s 	= 20			# track progress
	C_v 	= 10;			# ref velocity tracking weight			
	C_acc 	= 0
	C_dacc 	= 11;		    # 20 too high; 10 OK for med speed; 10 a big jerky for high speed; 13 too high
	v_min 	= -largeNumber 	# 0.0				# vel bounds (m/s)
	v_max 	=  largeNumber		# 20.0	
	a_max 	=  largeNumber	# 2.0				# acceleration and deceleration bound, m/s^2
	a_dmax 	= 1.5			# jerk bound, m/s^3

	# lateral parameters
	nx_lat 		= 2						# dimension of x = (ey,epsi)
	nu_lat 		= 1						# number of inputs u = df
	C_ey 		= 5.0					# lateral deviation
    C_epsi 		= 0.0
	C_df	 	= 0.0					# 150			# tire angle input
	C_ddf	 	= 1000.0				# 3e4			# derivative of tire angle input
	df_max 		= largeNumber			# 0.5					# steering
	ddf_max 	= 0.5					# change in steering

	NNgapThreshold_long 	= 1000.0			#  float64
	NNgapThreshold_lat		= 5.0			#  float64

    
	# LONG random data generation	
	aprev_lb 	= -2
	aprev_ub 	= 2
	ds_lb 		= 1			# from exp and sim data
	ds_ub		= 4				# from exp and sim data
	dv_lb 		= -0.5
	dv_ub  		= 0.5

	# LAT random data generation
	ey_lb       = -1					# -1.5
	ey_ub		=  1					# 1.5	
	epsi_lb     = -0.1					# -0.2 (previous 0.1)
	epsi_ub		=  0.1					#  0.2 (previous 0.1) 	
	vpred_lb    =  0
	vpred_ub    =  12					# (previous 12) 	
	dfprev_ub	=  0.5      			# (previous 0.5)
	dfprev_lb	= -0.5					# (previous 0.6)
	curv_lb  	=  -0.1
	curv_ub 	=   0.1					# +-0.05 from traj data			
	dcurv_lb 	= -0.015
	dcurv_ub 	=  0.015
end
