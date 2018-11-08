#!/usr/bin/env julia

# MPC Command Publisher/Controller Module Interface to the Genesis.
# This version using the Linear Kinematic Bicycle Model.

###########################################
#### ROBOTOS
###########################################
using RobotOS
@rosimport genesis_path_follower.msg: state_est
@rosimport genesis_path_follower.msg: mpc_path
@rosimport std_msgs.msg: UInt8
@rosimport std_msgs.msg: Float32
rostypegen()
using genesis_path_follower.msg
using std_msgs.msg
using PyCall


###########################################
#### LOAD ROSPARAMS
###########################################
if has_param("mat_waypoints")
	mat_fname = get_param("mat_waypoints")
else
	error("No Matfile of waypoints provided!")
end

if has_param("track_using_time") && has_param("target_vel")
	track_with_time = get_param("track_using_time")
	target_vel = get_param("target_vel")	
else
	error("Invalid rosparam trajectory definition: track_using_time and target_vel")
end

if has_param("scripts_dir")
	scripts_dir = get_param("scripts_dir")
else
	error("Did not provide the scripts directory!")
end

if(has_param("lat0") && has_param("lat0") && has_param("lat0"))
	lat0 = get_param("lat0")
	lon0 = get_param("lon0")
	yaw0 = get_param("yaw0")
else
	error("Invalid rosparam global origin provided!")
end

###########################################
#### MPC Controller Module with Cost Function Weights.
#### Global Variables for Callbacks/Control Loop.
###########################################
push!(LOAD_PATH, scripts_dir * "mpc_utils")

import KinMPCParams


import GPSKinMPCPathFollowerFrenetLinLatGurobi 	# don't really need them, only need them for 
import GPSKinMPCPathFollowerFrenetLinLongGurobi

import GPSKinMPCPathFollowerFrenetLinLatNN
import GPSKinMPCPathFollowerFrenetLinLongNN




# kmpcLinLongGurobi
# kmpcLinLatGurobi

# make sure the parameters (N, L, ...) are the same in both controllers
const kmpcLinLongNN = GPSKinMPCPathFollowerFrenetLinLongNN  # remaining
const kmpcLinLatNN = GPSKinMPCPathFollowerFrenetLinLatNN 	# experimental

###########################################
#### Reference GPS Trajectory Module
###########################################
# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
using PyCall
const gps_utils_loc = scripts_dir * "gps_utils"
unshift!(PyVector(pyimport("sys")["path"]), gps_utils_loc) # append the current directory to Python path
@pyimport ref_gps_traj as rgt
grt = rgt.GPSRefTrajectory(mat_filename=mat_fname, LAT0=lat0, LON0=lon0, YAW0=yaw0, traj_horizon = KinMPCParams.N, traj_dt=KinMPCParams.dt)

# Reference for MPC
const t_ref = collect(0:kmpcLinLongNN.dt:kmpcLinLongNN.N*kmpcLinLongNN.dt) 	 # array of length (N+1)
x_ref = zeros(length(t_ref))
y_ref = zeros(length(t_ref))
psi_ref = zeros(length(t_ref))
s_ref = zeros(length(t_ref))

# store the optimal input/steering (for deltaU formulation)
a_opt = 0.0
df_opt = 0.0

if target_vel > 0.0
	des_speed = target_vel
else
	des_speed = 0.00
end

ref_lock = false	# avoid racing
received_reference = false 		#TODO: can use time from last reading to see if data is fresh for MPC update.
x_curr  = 0.0
y_curr  = 0.0
psi_curr  = 0.0
v_curr  = 0.0
s_curr = 0.0		# frenet
ey_curr = 0.0		# frenet
epsi_curr = 0.0		# frenet
K_coeff = zeros(4)	# assuming curvature is described by polynomial of order 3


command_stop = false

###########################################
#### State Estimation Callback.
###########################################
function state_est_callback(msg::state_est)

	global x_curr, y_curr, psi_curr, v_curr
	global received_reference

	# if ref_lock == false
		x_curr = msg.x
		y_curr = msg.y
		psi_curr = msg.psi
		v_curr = msg.v
		received_reference = true
	# end
end


# x_mpc, y_mpc, psi_mpc = convertS2XY(res)	# this function converts the (s,c(s)) to (X,Y)
function convertS2XY(acc_opt, d_f_opt)
	# either use transformation from (s,c(s)) -> (x,y)
	# alternatively, could just use kin. model to forward integrate in time
	# X = X_des - e_y * sin(psi), Y = Y_des + e_y*cos(psi)
	global x_ref, y_ref, psi_ref, x_curr, y_curr, psi_curr, v_curr
	# println("entered convertS2XY function")
	x_mpc = zeros(KinMPCParams.N+1)
	y_mpc = zeros(KinMPCParams.N+1)
	psi_mpc = zeros(KinMPCParams.N+1)
	v_mpc = zeros(KinMPCParams.N+1)
	bta = zeros(KinMPCParams.N)
	# init initial state
	x_mpc[1] = x_curr
	y_mpc[1] = y_curr
	psi_mpc[1] = psi_curr
	v_mpc[1] = v_curr

	acc = acc_opt
	d_f = d_f_opt 	# TO BE OVER-WRITTEN


	for i in 1:KinMPCParams.N
        # equations of motion wrt CoG
        bta[i] = atan( KinMPCParams.L_b / (KinMPCParams.L_a + KinMPCParams.L_b) * tan(d_f[i]) )
		x_mpc[i+1]   = x_mpc[i]   + KinMPCParams.dt*( v_mpc[i]*cos(psi_mpc[i] + bta[i]) ) 
		y_mpc[i+1]   = y_mpc[i]   + KinMPCParams.dt*( v_mpc[i]*sin(psi_mpc[i] + bta[i]) ) 
		psi_mpc[i+1] = psi_mpc[i] + KinMPCParams.dt*( v_mpc[i]/KinMPCParams.L_b*sin(bta[i]) ) 
        v_mpc[i+1]   = v_mpc[i]   + KinMPCParams.dt*( acc[i] )
	end

	return x_mpc, y_mpc, v_mpc, psi_mpc
end


# function compures reference points from curvature profile
function compRefXYfromCurv(s0, s_max, k_coeff, hor)
	# either use transformation from (s,c(s)) -> (x,y)
	# alternatively, could just use kin. model to forward integrate in time
	# X = X_des - e_y * sin(psi), Y = Y_des + e_y*cos(psi)
	global x_ref, y_ref, psi_ref
	
	ds = 0.25
	s_interp = collect(0:ds:s_max)
	x_interp = zeros(length(s_interp))
	y_interp = zeros(length(s_interp))
	psi_interp = zeros(length(s_interp))

	x_interp[1] = x_ref[1]
	y_interp[1] = y_ref[1]
	psi_interp[1] = psi_ref[1]

	for i = 2 : length(s_interp)
		x_interp[i] = x_interp[i-1] + ds * cos(psi_interp[i-1])
		y_interp[i] = y_interp[i-1] + ds * sin(psi_interp[i-1])

		k_tmp = k_coeff[1]*(s0+s_interp[i-1])^3 + k_coeff[2]*(s0+s_interp[i-1])^2 + k_coeff[3]*(s0+s_interp[i-1]) + k_coeff[4]
		psi_interp[i] = psi_interp[i-1] + ds*k_tmp
	end

	x_recon = collect(0:0.5:10)
	y_recon = collect(0:0.5:10)
	psi_recon = collect(0:0.5:10)

	tmp = Int( round( length(s_interp) / hor))

	return x_interp[1:max(1,tmp):end], y_interp[1:max(1,tmp):end], psi_interp[1:max(1,tmp):end]
end
	

# publishing loop
function pub_loop(acc_pub_obj, steer_pub_obj, mpc_path_pub_obj)

	# println("start pub_loop")

	control_rate = 20 	# max 50 Hz
    loop_rate = Rate(control_rate)

	solv_time_long_gurobi1_all = zeros(control_rate/10*6000)		# over Gurobi.jl interface; mean: 2ms, max 4ms
	solv_time_lat_gurobi1_all = zeros(control_rate/10*6000)		# over Gurobi.jl interface; mean: 2ms, max 4ms
	solv_time_gurobi_tot_all = zeros(control_rate/10*6000)

	absDualGap = zeros(control_rate/10*6000)		# primal - dualObj 
	absDualGapMod = zeros(control_rate/10*6000) 	# primal - max(dualObj,0)
	relDualGap = zeros(control_rate/10*6000)
	dualObj = zeros(control_rate/10*6000)


	num_NN_long = 0		# counter for how often NN was called
	num_NN_lat = 0

    num_warmStarts = 2	# number of warmstarts - no control applied during these steps
    it_num = 0	# iteration_count

	gc()	# clear garbage
	# can disable GC here, but memory consumption grows very quickly
	# gc_enable(false) # enable later on

    while ! is_shutdown()
	    if ! received_reference		# Reference not received so don't use MPC yet.
	        rossleep(loop_rate)
	        continue
	    end

	    global ref_lock				# Ref lock used to ensure that get/set of state doesn't happen simultaneously.
	    ref_lock = true

		global x_curr, y_curr, psi_curr, v_curr, des_speed, command_stop
		global s_ref, K_coeff, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref
		global a_opt, df_opt 	# contains current optimal inut and steering angle

		if ! track_with_time		
			# x_ref, y_ref, psi_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr, des_speed)
			s_ref, K_coeff, stop_cmd, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref, v_ref = grt[:get_waypoints_frenet](x_curr, y_curr, psi_curr)
			println("get_waypoints_frenet with velocity tracking not yet implemented - using time-tracking")
			if stop_cmd == true
				command_stop = true
			end

		else  # current version, time-based
			# x_ref, y_ref, psi_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr);
			# return reference points as well as current state (in Frenet framework)
			# last three only used for visualization purposesbt
			s_ref, K_coeff, stop_cmd, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref, v_ref = grt[:get_waypoints_frenet](x_curr, y_curr, psi_curr)
			if stop_cmd == true
				command_stop = true
			end
		end


		# fix epsi_curr if not good (thanks, Vijay!)
		if epsi_curr > pi
			epsi_curr = epsi_curr - 2*pi
		elseif epsi_curr < -pi
			epsi_curr = epsi_curr + 2*pi
		end


	    ref_lock = false
		
		if command_stop == false

			# disable garbage collection (makes optimization code run faster)
			gc_enable(false) # enable later on

			# println("================  iteration $(it_num) =====================")

			# a_opt=u0 ; a_pred = (u_0, u_1, ... u_{N-1})
			a_opt_gurobi, a_pred_gurobi, s_pred_gurobi, v_pred_gurobi, dA_pred_gurobi, solv_time_long_gurobi1, is_opt_long, solMode_long, primNN_obj, dualNN_obj, xu_tilde_NN_res = kmpcLinLongNN.get_NNsolution(s_curr, v_curr, a_opt, s_ref, v_ref)
			
			# a_opt_gurobi, a_pred_gurobi, s_pred_gurobi, v_pred_gurobi, dA_pred_gurobi, solv_time_long_gurobi1, is_opt_long = GPSKinMPCPathFollowerFrenetLinLongGurobi.solve_gurobi(s_curr, v_curr, a_opt, s_ref, v_ref)
			# solMode_long = " "
			# primNN_obj = 0
			# dualNN_obj = 0
			# xu_tilde_NN_res = 0
			dualObj[it_num+1] = dualNN_obj[1]
			absDualGap[it_num+1] = primNN_obj[1] - dualNN_obj[1]
			absDualGapMod[it_num+1] = primNN_obj[1] - max(dualNN_obj[1],0)
			# relDualGap[it_num+1]  = (primNN_obj[1] - dualNN_obj[1]) / mean([primNN_obj[1], dualNN_obj[1]])





			if (is_opt_long==1) && (solMode_long=="NN")
				num_NN_long = num_NN_long + 1
			end
			solv_time_long_gurobi1_all[it_num+1] = solv_time_long_gurobi1


			df_opt_gurobi, df_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, ddf_pred_gurobi, solv_time_lat_gurobi1, is_opt_lat, solMode_lat, primNN_Lat_obj, dualNN_lat_obj, xu_tilde_lat_NN_res = kmpcLinLatNN.get_NNsolution(ey_curr, epsi_curr, df_opt, s_pred_gurobi, v_pred_gurobi, K_coeff)
			# df_opt_gurobi, df_pred_gurobi, ddf_pred_gurobi, ey_pred_gurobi, epsi_pred_gurobi, solv_time_lat_gurobi1, is_opt_lat = kmpcLinLatNN.solve_gurobi(ey_curr, epsi_curr, df_opt, s_pred_gurobi, v_pred_gurobi, K_coeff)
			
			if (is_opt_lat==1) && (solMode_lat=="NN")
				num_NN_lat = num_NN_lat + 1
			end

			# num_NN_lat
			solv_time_lat_gurobi1_all[it_num+1] = solv_time_lat_gurobi1



			rostm = get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs

			log_str_long_Gurobi = @sprintf("Solve Status Long. Gurobi.: %s, Acc: %.3f, SolvTimeLong Gurobi:  %.3f", is_opt_long,  a_opt_gurobi, solv_time_long_gurobi1)
			loginfo(log_str_long_Gurobi)

			log_str_lat = @sprintf("Solve Status Lat. Gurobi: %s, SA: %.3f, SolvTimeLat:  %.3f", is_opt_lat, df_opt_gurobi, solv_time_lat_gurobi1)
		    loginfo(log_str_lat)



		    a_prev = a_opt    		# store previous acceleration (not super clean)
			a_opt = a_opt_gurobi	# update new acceleration
			df_prev = df_opt		# store previous acceleration
			df_opt = df_opt_gurobi 	# update new acceleration

		    # do not apply any inputs during warm start
		    if it_num <= num_warmStarts
		    	it_num = it_num + 1;
		    	continue					
		    end



			publish( acc_pub_obj,   Float32Msg(a_opt) )
			publish( steer_pub_obj, Float32Msg(df_opt) )
			# publish( solv_pub_obj, Float32Msg(solv_time_long_gurobi1 + solv_time_lat_gurobi1) )
    		
    		solv_time_gurobi_tot_all[it_num+1] = solv_time_long_gurobi1 + solv_time_lat_gurobi1

    		# reconstruct x_ref and y_ref from s,c(s)
			x_ref_recon, y_ref_recon, psi_ref_recon = compRefXYfromCurv(s_curr, s_ref[end]-s_ref[1], K_coeff, length(s_ref)-1)

			# compute the predicted x,y 
			x_mpc, y_mpc, v_mpc, psi_mpc = convertS2XY(a_pred_gurobi, df_pred_gurobi)	# this function converts the (s,c(s)) to (X,Y)


			# save relevant messages
			mpc_path_msg = mpc_path()
			mpc_path_msg.header.stamp = rostm
			mpc_path_msg.solv_status_long  = string(is_opt_long)
			mpc_path_msg.solv_status_lat  = string(is_opt_lat)
			mpc_path_msg.solv_time_long = solv_time_long_gurobi1
			mpc_path_msg.solv_time_lat = solv_time_lat_gurobi1
			# store current states and previous inputs (for NN learning)
			mpc_path_msg.s_curr = s_curr
			mpc_path_msg.v_curr = v_curr
			mpc_path_msg.a_prev = a_prev
			mpc_path_msg.ey_curr = ey_curr
			mpc_path_msg.epsi_curr = epsi_curr
			mpc_path_msg.df_prev = df_prev
			# store the predicted paths in (X,Y)
			mpc_path_msg.xs   = x_mpc 	# x_mpc; containts x_curr
			mpc_path_msg.ys   = y_mpc 	# y_mpc; containts y_curr
			mpc_path_msg.vs   = v_mpc	# v_mpc; containts v_curr
			mpc_path_msg.psis = psi_mpc 	# psi_mpc; containts psi_curr
			# Optimal solution of MPC problem (in Frenet)
			mpc_path_msg.ss_fren   	= s_pred_gurobi		# contains s_0 
			mpc_path_msg.vs_fren   	= v_pred_gurobi		# contains v_0
			mpc_path_msg.eys_fren   = ey_pred_gurobi	# contains ey_0
			mpc_path_msg.epsis_fren = epsi_pred_gurobi 	# contains epsi_0
			mpc_path_msg.curv 		= K_coeff
			# add reference trajectory in X,Y coordinates
			mpc_path_msg.xr   = x_ref 	# x_ref
			mpc_path_msg.yr   = y_ref 	# y_ref
			mpc_path_msg.vr   = v_ref	# v_ref; contains v0
			mpc_path_msg.sr   = s_ref	# s_ref; contains s0; for policy-learning
			mpc_path_msg.psir = psi_ref 	# psi_ref
			# "ref"-path reconstructed from Frenet
			mpc_path_msg.xr_recon   = x_ref_recon 	# x_ref
			mpc_path_msg.yr_recon   = y_ref_recon 	# y_ref
			mpc_path_msg.vr_recon   = v_ref	# v_ref
			mpc_path_msg.psir_recon = psi_ref_recon 	# psi_ref
			# # store current and predicted inputs
			mpc_path_msg.acc   = a_pred_gurobi	# d_f
			mpc_path_msg.df  = df_pred_gurobi	# acc
			mpc_path_msg.ddf = ddf_pred_gurobi   # delta df; for policy-learning
			mpc_path_msg.dacc = dA_pred_gurobi	# delta ACC; for policy-learning



			publish(mpc_path_pub_obj, mpc_path_msg)




	    	it_num = it_num + 1		# iteration_count

			# println("--- max comput time Long IPOPT: $(maximum(solv_time_long_ipopt_all[5:end])*1000) ms ---")
			# println("avg comput time Long IPOPT: $(mean(solv_time_long_ipopt_all[5:it_num-1])*1000) ms")
			# println("solv times Long IPOPT more than 15ms: $(sum(solv_time_long_ipopt_all[5:end] .> 15e-3*ones(1000-4) ))")

			if mod(it_num,50)==0
				println("================  iteration $(it_num) =====================")
				println("--- max comput time Long GUROBI: $(maximum(solv_time_long_gurobi1_all[5:end])*1000) ms ---")
				println("avg comput time Long GUROBI: $(mean(solv_time_long_gurobi1_all[5:it_num-1])*1000) ms")
				# println("solv times Long GUROBI 1 more than 15ms: $(sum(solv_time_long_gurobi1_all[5:end] .> 15e-3*ones(1000-4) ))")
				println("--- max comput time Lat GUROBI: $(maximum(solv_time_lat_gurobi1_all[5:end])*1000) ms ---")
				println("avg comput time Lat GUROBI: $(mean(solv_time_lat_gurobi1_all[5:it_num-1])*1000) ms")
				# println("--- max comput time Lat Ipopt: $(maximum(solv_time_lat_ipopt_all[5:end])*1000) ms ---")
				# println("avg comput time Lat Ipopt: $(mean(solv_time_lat_ipopt_all[5:it_num-1])*1000) ms")
				println("--- max comput time tot GUROBI: $(maximum(solv_time_gurobi_tot_all[5:end])*1000) ms ---")
				println(" avg comput time tot GUROBI: $(mean(solv_time_gurobi_tot_all[5:end])*1000) ms")

				println("--- percentage of NN long called: $(num_NN_long/it_num*100) %---")
				println("--- percentage of NN lat called: $(num_NN_lat/it_num*100) %---")
				
				println(" ")

				println("--- percentage dual_obj < 0: $( sum(dualObj[1:it_num-1].<0)/(it_num-1)*100 ) %")

				println(" ")
				
				println("--- max abs primal-dual NN gap: $(maximum(absDualGap[1:it_num-1]))")
				println("avg abs primal-dual NN gap: $(mean(absDualGap[1:it_num-1]))")
				println("median abs primal-dual NN gap: $(median(absDualGap[1:it_num-1]))")
				
				println(" ")
				
				println("--- max abs mod primal-dual NN gap: $(maximum(absDualGapMod[1:it_num-1]))")
				println("avg abs mod primal-dual NN gap: $(mean(absDualGapMod[1:it_num-1]))")
				println("median abs mod primal-dual NN gap: $(median(absDualGapMod[1:it_num-1]))")

				println(" ")
				
				println("--- frac of abs gap below 1: $( sum(absDualGap[1:it_num-1].<=1)/(it_num-1)*100 ) %")
				println("frac of abs gap below 2: $( sum(absDualGap[1:it_num-1].<=2)/(it_num-1)*100 ) %")
				println("frac of abs gap below 3: $( sum(absDualGap[1:it_num-1].<=3)/(it_num-1)*100 ) %")
				println("frac of abs gap below 4: $( sum(absDualGap[1:it_num-1].<=4)/(it_num-1)*100 ) %")
				println("frac of abs gap below 5: $( sum(absDualGap[1:it_num-1].<=5)/(it_num-1)*100 ) %")
				println("frac of abs gap below 6: $( sum(absDualGap[1:it_num-1].<=6)/(it_num-1)*100 ) %")
				println("frac of abs gap below 7: $( sum(absDualGap[1:it_num-1].<=7)/(it_num-1)*100 ) %")
				println("frac of abs gap below 8: $( sum(absDualGap[1:it_num-1].<=8)/(it_num-1)*100 ) %")
				println("frac of abs gap below 9: $( sum(absDualGap[1:it_num-1].<=9)/(it_num-1)*100 ) %")
				println("frac of abs gap below 10: $( sum(absDualGap[1:it_num-1].<=10)/(it_num-1)*100 ) %")

				println(" ")
				
				println("--- frac of abs mod gap below 1: $( sum(absDualGapMod[1:it_num-1].<=1)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 2: $( sum(absDualGapMod[1:it_num-1].<=2)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 3: $( sum(absDualGapMod[1:it_num-1].<=3)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 4: $( sum(absDualGapMod[1:it_num-1].<=4)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 5: $( sum(absDualGapMod[1:it_num-1].<=5)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 6: $( sum(absDualGapMod[1:it_num-1].<=6)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 7: $( sum(absDualGapMod[1:it_num-1].<=7)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 8: $( sum(absDualGapMod[1:it_num-1].<=8)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 9: $( sum(absDualGapMod[1:it_num-1].<=9)/(it_num-1)*100 ) %")
				println("frac of abs mod gap below 10: $( sum(absDualGapMod[1:it_num-1].<=10)/(it_num-1)*100 ) %")


				println(" ")

				println("--- max rel primal-dual NN gap: $(maximum(relDualGap[1:it_num-1]))")
				println("--- avg rel primal-dual NN gap: $(mean(relDualGap[1:it_num-1]))")
				println("--- min rel primal-dual NN gap: $(minimum(relDualGap[1:it_num-1]))")

			end
			


			# println("--- max comput time Long GUROBI 2: $(maximum(solv_time_long_gurobi2_all[5:end])*1000) ms ---")
			# println("avg comput time Long GUROBI 2: $(mean(solv_time_long_gurobi2_all[5:it_num-1])*1000) ms")
			# println("solv times Long GUROBI 2 more than 15ms: $(sum(solv_time_long_gurobi2_all[5:end] .> 15e-3*ones(1000-4) ))")


			# println("--- max comput time Long OSQP: $(maximum(solv_time_long_osqp_all[5:end])*1000) ms ---")
			# println("avg comput time Long OSQP: $(mean(solv_time_long_osqp_all[5:it_num-1])*1000) ms")
			# println("solv times Long OSQP more than 15ms: $(sum(solv_time_long_osqp_all[5:end] .> 15e-3*ones(1000-4) ))")

			# re-enable garbage collection
			gc_enable(true)

		   	# println("max comput time Long GUROBI: $(maximum(solv_time_long_gurobi_all[5:end]))")
		   	# println("max setup time Long GUROBI: $(maximum(setup_time_long_gurobi_all[5:end]))")
		   	# println("setup time Long GUROBI more than 5ms: $(sum(setup_time_long_gurobi_all[5:end] .> 5e-3*ones(1000-4) )) ---")
		   	# println("max pure solv time Long GUROBI: $(maximum(purSolv_time_long_gurobi_all[5:end]))")

		   	# println("max NEW solv time Long GUROBI: $(maximum(newSolv_time_long_gurobi_all[5:end]))")

		   	# println("max  solv time Long OSQP: $(maximum(newSolv_time_long_osqp_all[5:end]))")
		   	# println("--- solv time Long OSQP more than 5ms: $(sum(newSolv_time_long_osqp_all[5:end] .> 5e-3*ones(1000-4) )) ---")

		else
			publish( acc_pub_obj,   Float32Msg(-1.0) )
			publish( steer_pub_obj, Float32Msg(0.0) )		
		end
		
	    rossleep(loop_rate)
	end

	# gc_enable(true) 

end	

function start_mpc_node()
    init_node("dbw_mpc_pf")
    acc_pub   = Publisher("/control/accel", Float32Msg, queue_size=2)
    steer_pub = Publisher("/control/steer_angle", Float32Msg, queue_size=2)

    acc_enable_pub   = Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=true)
    steer_enable_pub = Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=true)

    mpc_path_pub = Publisher("mpc_path", mpc_path, queue_size=2)
	sub_state  = Subscriber("state_est", state_est, state_est_callback, queue_size=2)

	publish(acc_enable_pub, UInt8Msg(2))
	publish(steer_enable_pub, UInt8Msg(1))

    pub_loop(acc_pub, steer_pub, mpc_path_pub)    
end

if ! isinteractive()
	# try 
    	start_mpc_node()
    # catch x
    	# println("222")
    	# print(x)
    # end
end
