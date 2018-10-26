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

	dt      = 0.20			# model discretization time, td (s)
	# dt 		= 0.1

	N       = 8				# horizon
	# N		= 16

	L_a 	= 1.5213		# from CoG to front axle (according to Jongsang)
	L_b 	= 1.4987		# from CoG to rear axle (according to Jongsang)

end
