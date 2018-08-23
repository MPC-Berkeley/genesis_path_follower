#!/usr/bin/env julia

# MPC Command Publisher/Controller Module Interface to the Genesis.
# This version using a Linearized Dynamic Bicycle Model - in progress.

###########################################
#### ROBOTOS
###########################################
using RobotOS
@rosimport genesis_path_follower.msg: state_est_dyn
@rosimport genesis_path_follower.msg: mpc_path_dyn_frenet
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
import GPSDynMPCPathFollowerFrenetLinLongGurobi 
import GPSDynMPCPathFollowerFrenetLinLatGurobi
import DynBicycleModel

# make sure the parameters (N, L, ...) are the same in both controllers
const dmpcLinLongGurobi = GPSDynMPCPathFollowerFrenetLinLongGurobi
const dmpcLinLatGurobi = GPSDynMPCPathFollowerFrenetLinLatGurobi
const dyn_mdl = DynBicycleModel.f_dyn_bicycle_model

###########################################
#### Reference GPS Trajectory Module
###########################################
# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
using PyCall
const gps_utils_loc = scripts_dir * "gps_utils"
unshift!(PyVector(pyimport("sys")["path"]), gps_utils_loc) # append the current directory to Python path
@pyimport ref_gps_traj_dyn_frenet as rgt
grt = rgt.GPSRefTrajectory(mat_filename=mat_fname, LAT0=lat0, LON0=lon0, YAW0=yaw0, traj_horizon = dmpcLinLongGurobi.N, traj_dt = dmpcLinLongGurobi.dt)

# Reference for MPC
const t_ref = collect(0:dmpcLinLongGurobi.dt:dmpcLinLongGurobi.N*dmpcLinLongGurobi.dt)  # array of length (N+1)
x_ref = zeros(length(t_ref))
y_ref = zeros(length(t_ref))
psi_ref = zeros(length(t_ref))
vx_ref = zeros(length(t_ref))
vy_ref = zeros(length(t_ref))
wz_ref = zeros(length(t_ref))

# store optimal input/steering (for delta-Input formulation)
# a_opt = 0.0
# df_opt = 0.0

if target_vel > 0.0
	des_speed = target_vel
else
	des_speed = 0.00
end

ref_lock 			= false				
received_reference 	= false
command_stop 		= false

# states in (X,Y)-coordinates
x_curr  			= 0.0
y_curr  			= 0.0
psi_curr  			= 0.0
v_curr  			= 0.0
vx_curr  			= 0.0
vy_curr  			= 0.0
wz_curr  			= 0.0

# addition states required by frenet coordinates
s_curr 				= 0.0
ey_curr				= 0.0
epsi_curr			= 0.0
K_coeff 			= zeros(4)	# assuming curvature is described by polynomial of order 3


# previous inputs - used for Delta-Input formulation
prev_a 				= 0.0 		
prev_df 			= 0.0

###########################################
#### State Estimation Callback.
###########################################
function state_est_callback(msg::state_est_dyn)

	global x_curr, y_curr, psi_curr, v_curr, vx_curr, vy_curr, wz_curr
	global received_reference

	# if ref_lock == false
		x_curr = msg.x
		y_curr = msg.y
		psi_curr = msg.psi
		v_curr = msg.v
		vx_curr = msg.vx
		vy_curr = msg.vy
		wz_curr = msg.wz
		received_reference = true
	# end
end

function convertS2XY(acc, d_f, z_curr)
   # states contain current states as well
   x_mpc   = zeros(dmpcLinLongGurobi.N+1)
   y_mpc   = zeros(dmpcLinLongGurobi.N+1)
   psi_mpc = zeros(dmpcLinLongGurobi.N+1)
   v_mpc = zeros(dmpcLinLongGurobi.N+1)
   bta = zeros(dmpcLinLongGurobi.N)	

    # init initial state
   x_mpc[1]   = z_curr[1]
   y_mpc[1]   = z_curr[2]
   psi_mpc[1] = z_curr[3]
   v_mpc[1]   = z_curr[4]

   for i in 1:dmpcLinLongGurobi.N
    # equations of motion wrt CoG
    bta[i] = atan( dmpcLinLongGurobi.L_b / (dmpcLinLongGurobi.L_a + dmpcLinLongGurobi.L_b) * tan(d_f[i]) )
           x_mpc[i+1]   = x_mpc[i]   + dmpcLinLongGurobi.dt*( v_mpc[i]*cos(psi_mpc[i] + bta[i]) ) 
           y_mpc[i+1]   = y_mpc[i]   + dmpcLinLongGurobi.dt*( v_mpc[i]*sin(psi_mpc[i] + bta[i]) ) 
           psi_mpc[i+1] = psi_mpc[i] + dmpcLinLongGurobi.dt*( v_mpc[i]/dmpcLinLongGurobi.L_b*sin(bta[i]) ) 
	       v_mpc[i+1]   = v_mpc[i]   + dmpcLinLongGurobi.dt*( acc[i] )
    end
	return x_mpc, y_mpc, psi_mpc, v_mpc
end

function convertS2XYdyn(acc, d_f, z_curr)
	# states contain current states as well
	x_mpc   = zeros(dmpcLinLongGurobi.N+1)
	y_mpc   = zeros(dmpcLinLongGurobi.N+1)
	psi_mpc = zeros(dmpcLinLongGurobi.N+1)
	vx_mpc  = zeros(dmpcLinLongGurobi.N+1)
	vy_mpc  = zeros(dmpcLinLongGurobi.N+1)
	wz_mpc  = zeros(dmpcLinLongGurobi.N+1)

	# init initial state
	x_mpc[1]   = z_curr[1]
	y_mpc[1]   = z_curr[2]
	psi_mpc[1] = z_curr[3]
	vx_mpc[1]  = z_curr[4]
	vy_mpc[1]  = z_curr[5]
	wz_mpc[1]  = z_curr[6]

	for i in 1:dmpcLinLongGurobi.N
        zu_curr = vec([z_curr; acc[i]; d_f[i]])
		f = dyn_mdl(zu_curr)
		z_curr = convert(Array{Float64,1}, z_curr + f * dmpcLinLongGurobi.dt )

		x_mpc[i+1]   = z_curr[1]
		y_mpc[i+1]   = z_curr[2]
		psi_mpc[i+1] = z_curr[3]
		vx_mpc[i+1]  = z_curr[4]
		vy_mpc[i+1]  = z_curr[5]
		wz_mpc[i+1]  = z_curr[6]
	end

	return x_mpc, y_mpc, psi_mpc, vx_mpc, vy_mpc, wz_mpc
end

# function compures reference points from curvature profile
function compRefXYfromCurv(s0, s_max, k_coeff, hor)
	# either use transformation from (s,c(s)) -> (x,y)
	# alternatively, could just use kin. model to forward integrate in time
	# X = X_des - e_y * sin(psi), Y = Y_des + e_y*cos(psi)
	global x_ref, y_ref, psi_ref
	
	ds = 0.25 	# integration step
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

	# downsample: get N-predictions
	tmp = Int( round( length(s_interp) / hor))

	return x_interp[1:max(1,tmp):end], y_interp[1:max(1,tmp):end], psi_interp[1:max(1,tmp):end]
end



function pub_loop(acc_pub_obj, steer_pub_obj, mpc_path_pub_obj)

	control_rate = 50 	# max 50Hz
    loop_rate = Rate(control_rate)

    solv_time_long_all = zeros(control_rate/10*6000)		# over Gurobi.jl interface; mean: 2ms, max 4ms
	solv_time_lat_all = zeros(control_rate/10*6000)		# over Gurobi.jl interface; mean: 2ms, max 4ms
	solv_time_tot_all = zeros(control_rate/10*6000)

	num_warmStarts = 2 	# number of warm starts (no control applied during these steps)
	it_num = 0	# iteration_count

	gc()		# clear garbage

	# gc_enable(false)
    
    while ! is_shutdown()
	    if ! received_reference		# Reference not received so don't use MPC yet.
	        rossleep(loop_rate)
	        continue
	    end

	    global ref_lock				# Ref lock used to ensure that get/set of state doesn't happen simultaneously.
	    ref_lock = true

		global x_curr, y_curr, psi_curr, v_curr, vx_curr, vy_curr, wz_curr, des_speed, command_stop
		global s_ref, vx_ref, K_coeff, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref
		global prev_a, prev_df

		if ! track_with_time		
			# fixed velocity-based path tracking
			# GXZ: need to determine if more outputs are needed
			s_ref, K_coeff, stop_cmd, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref, vx_ref, vy_ref, wz_ref = grt[:get_waypoints_frenet](x_curr, y_curr, psi_curr)
			# x_ref, y_ref, psi_ref, vx_ref, vy_ref, wz_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr, des_speed)

			if stop_cmd == true
				command_stop = true
			end

		else
			# trajectory tracking
			# x_ref, y_ref, psi_ref, vx_ref, vy_ref, wz_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr)
			s_ref, K_coeff, stop_cmd, s_curr, ey_curr, epsi_curr, x_ref, y_ref, psi_ref, vx_ref, vy_ref, wz_ref = grt[:get_waypoints_frenet](x_curr, y_curr, psi_curr)

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

		# Update Model
		# z_ref = [x_ref y_ref psi_ref vx_ref vy_ref wz_ref]
		z_curr = vec([x_curr; y_curr; psi_curr; vx_curr; vy_curr; wz_curr])
		# u_curr = vec([prev_a; prev_df])
		# dmpc.update_reference(z_ref)
		# dmpc.update_model(z_curr, u_curr)

	    ref_lock = false
		
		if command_stop == false


			# dis-able garbage collection (makes optimization time more consistent)
			gc_enable(false)

			# a_opt, df_opt, is_opt, solv_time = dmpc.solve_model()

			# a_opt = u0 ; a_pred = (u_0, u_1, ... u_{N-1})
			a_opt, a_pred, s_pred, vx_pred, solv_time_long, is_opt_long = dmpcLinLongGurobi.solve_gurobi(s_curr, vx_curr, prev_a, s_ref, vx_ref)
			solv_time_long_all[it_num+1] = solv_time_long

			df_opt, df_pred, ey_pred, epsi_pred, vy_pred, wz_pred, solv_time_lat, is_opt_lat = dmpcLinLatGurobi.solve_gurobi(ey_curr, epsi_curr, vy_curr, wz_curr, prev_df, s_pred, vx_pred, K_coeff)
			solv_time_lat_all[it_num+1] = solv_time_lat

    		solv_time_tot_all[it_num+1] = solv_time_long + solv_time_lat


			rostm = get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs

		    log_str_long = @sprintf("Solve Status Long.: %s, Acc: %.3f, SolvTimeLong Gurobi:  %.3f", is_opt_long,  a_opt, solv_time_long)
			loginfo(log_str_long)

			log_str_lat = @sprintf("Solve Status Lat.: %s, SA: %.3f, SolvTimeLat:  %.3f", is_opt_lat, df_opt, solv_time_lat)
		    loginfo(log_str_lat)

			# do not apply any inputs during warm start
		    if it_num <= num_warmStarts
		    	it_num = it_num + 1;
		    	continue					
		    end

			publish( acc_pub_obj,   Float32Msg(a_opt) )
			publish( steer_pub_obj, Float32Msg(df_opt) )

			# store optimal inputs
			prev_a = a_opt
			prev_df = df_opt
			
			# res = dmpc.get_solver_results()

			# reconstruct x_ref and y_ref from s,c(s)
			# THIS FUNCTION SHOULD BE UPDATED WITH DYNAMIC MODEL
			x_ref_recon, y_ref_recon, psi_ref_recon = compRefXYfromCurv(s_curr, s_ref[end]-s_ref[1], K_coeff, length(s_ref)-1)

			# this function converts the (s,c(s)) to (X,Y)
			if z_curr[4] 
			x_mpc, y_mpc, psi_mpc, vx_mpc, vy_mpc, wz_mpc = convertS2XYdyn(a_pred, df_pred, z_curr) 			

			mpc_path_msg = mpc_path_dyn_frenet()
			mpc_path_msg.header.stamp = rostm			
			mpc_path_msg.solv_status_long = string(is_opt_long)
			mpc_path_msg.solv_status_lat  = string(is_opt_lat)
			mpc_path_msg.solv_time_long = solv_time_long
			mpc_path_msg.solv_time_lat = solv_time_lat

			# predicted states in X-Y coordinates
			mpc_path_msg.xs   = x_mpc 	# x_mpc
			mpc_path_msg.ys   = y_mpc 	# y_mpc
			mpc_path_msg.psis = psi_mpc # psi_mpc
			mpc_path_msg.vxs  = vx_mpc 	# vx_mpc
			mpc_path_msg.vys  = vy_mpc 	# vy_mpc
			mpc_path_msg.wzs  = wz_mpc 	# vz_mpc
			mpc_path_msg.ss_fren 	= s_pred
			mpc_path_msg.eys_fren	= ey_pred
			mpc_path_msg.epsis_fren = epsi_pred
			mpc_path_msg.curv 		= K_coeff

			mpc_path_msg.xr   = x_ref 		# x_ref
			mpc_path_msg.yr   = y_ref 		# y_ref
			mpc_path_msg.psir = psi_ref 	# psi_ref
			mpc_path_msg.vxr  = vx_ref 		# vx_ref
			mpc_path_msg.vyr  = vy_ref 		# vy_ref
			mpc_path_msg.wzr  = wz_ref 		# vz_ref



			mpc_path_msg.xr_recon 	= x_ref_recon
			mpc_path_msg.yr_recon   = y_ref_recon
			mpc_path_msg.psir_recon = psi_ref_recon
			# mpc_path_msg.vxr_recon =      			# unused
			# mpc_path_msg.vyr_recon = 					# unused
			# mpc_path_msg.wzr_recon = 					# unused

			mpc_path_msg.df   = df_pred	# d_f (vectors)
			mpc_path_msg.acc  = a_pred	# acc (vectors)
			
			publish(mpc_path_pub_obj, mpc_path_msg)

			it_num = it_num + 1 	# update iteration iteration_count

			if mod(it_num,100)==0
				println("================  iteration $(it_num) =====================")
				println("--- max comput time Long GUROBI: $(maximum(solv_time_long_all[5:end])*1000) ms ---")
				println("avg comput time Long GUROBI: $(mean(solv_time_long_all[5:it_num-1])*1000) ms")
				# println("solv times Long GUROBI 1 more than 15ms: $(sum(solv_time_long_gurobi1_all[5:end] .> 15e-3*ones(1000-4) ))")
				println("--- max comput time Lat GUROBI: $(maximum(solv_time_lat_all[5:end])*1000) ms ---")
				println("avg comput time Lat GUROBI: $(mean(solv_time_lat_all[5:it_num-1])*1000) ms")
				# println("--- max comput time Lat Ipopt: $(maximum(solv_time_lat_ipopt_all[5:end])*1000) ms ---")
				# println("avg comput time Lat Ipopt: $(mean(solv_time_lat_ipopt_all[5:it_num-1])*1000) ms")
				println("--- max comput time tot GUROBI: $(maximum(solv_time_tot_all[5:end])*1000) ms ---")
				println(" avg comput time tot GUROBI: $(mean(solv_time_tot_all[5:end])*1000) ms")
			end

			gc_enable(true)

		else
			publish( acc_pub_obj,   Float32Msg(-1.0) )
			publish( steer_pub_obj, Float32Msg(0.0) )		
		end
	    rossleep(loop_rate)
	end

	# gc_enable()
end	

function start_mpc_node()
    init_node("dbw_mpc_pf")
    acc_pub   = Publisher("/control/accel", Float32Msg, queue_size=2)
    steer_pub = Publisher("/control/steer_angle", Float32Msg, queue_size=2)

    acc_enable_pub   = Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=true)
    steer_enable_pub = Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=true)

    mpc_path_pub = Publisher("mpc_path_dyn", mpc_path_dyn_frenet, queue_size=2)
	sub_state  = Subscriber("state_est_dyn", state_est_dyn, state_est_callback, queue_size=2)    

	# Start up Ipopt/Solver.
	# for i = 1:3
	# 	dmpc.solve_model()
	# end

	publish(acc_enable_pub, UInt8Msg(2))
	publish(steer_enable_pub, UInt8Msg(1))

    pub_loop(acc_pub, steer_pub, mpc_path_pub)    
end

if ! isinteractive()
	# try 
    	start_mpc_node()
    # catch x
    	# print(x)
    # end
end
