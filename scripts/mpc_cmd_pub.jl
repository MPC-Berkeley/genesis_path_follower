#!/usr/bin/env julia

# ROS Imports
using RobotOS
@rosimport genesis_path_follower.msg: state_est
@rosimport genesis_path_follower.msg: mpc_path
@rosimport std_msgs.msg: UInt8
@rosimport std_msgs.msg: Float32
rostypegen()
using genesis_path_follower.msg
using std_msgs.msg
using PyCall

csv_fname = PyObject(nothing)
pkl_fname = PyObject(nothing)
track_with_time = false
target_vel = 0.0

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

scripts_dir = ""
if has_param("scripts_dir")
	scripts_dir = get_param("scripts_dir")
else
	error("Did not provide the scripts directory!")
end

# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
using PyCall
const gps_utils_loc = scripts_dir * "gps_utils"
unshift!(PyVector(pyimport("sys")["path"]), gps_utils_loc) # append the current directory to Python path
@pyimport ref_gps_traj as rgt
grt = rgt.GPSRefTrajectory(mat_filename=mat_fname)

# Access MPC Controller.
push!(LOAD_PATH, scripts_dir * "mpc_utils")
import GPSKinMPCPathFollower
const kmpc = GPSKinMPCPathFollower

kmpc.update_cost(9.0, 9.0, 10.0, 0.0, 100.0, 1000.0, 0.0, 0.0) # x,y,psi,v,da,ddf,a,df

const t_ref = collect(0:kmpc.dt:kmpc.N*kmpc.dt)
x_ref = zeros(length(t_ref))
y_ref = zeros(length(t_ref))
psi_ref = zeros(length(t_ref))

received_reference = false 		#TODO: can use time from last reading to see if data is fresh for MPC update.

if target_vel > 0.0
	des_speed = target_vel
else
	des_speed = 0.00
end

ref_lock = false
x_curr  = 0.0
y_curr  = 0.0
psi_curr  = 0.0
v_curr  = 0.0

command_stop = false

function state_est_callback(msg::state_est)

	global x_curr, y_curr, psi_curr, v_curr
	global received_reference

	if ref_lock == false
		x_curr = msg.x
		y_curr = msg.y
		psi_curr = msg.psi
		v_curr = msg.v
		received_reference = true
	end
end

function pub_loop(acc_pub_obj, steer_pub_obj, path_pub_obj, mpc_path_pub_obj)
    loop_rate = Rate(10.0)
    while ! is_shutdown()
	    if ! received_reference
	        rossleep(loop_rate)
	        continue
	    end

	    global ref_lock
	    ref_lock = true

		global x_curr, y_curr, psi_curr, v_curr, des_speed, command_stop

		if ! track_with_time		
			x_ref, y_ref, psi_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr, des_speed)

			if stop_cmd == true
				command_stop = true
			end

		else
			x_ref, y_ref, psi_ref, stop_cmd = grt[:get_waypoints](x_curr, y_curr, psi_curr)

			if stop_cmd == true
				command_stop = true
			end
		end
		
		# Update Model
		kmpc.update_init_cond(x_curr, y_curr, psi_curr, v_curr)
		kmpc.update_reference(x_ref, y_ref, psi_ref, des_speed)

	    ref_lock = false
		
		if command_stop == false
			a_opt, df_opt, is_opt = kmpc.solve_model()

			rostm = get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs

		    log_str = @sprintf("Solve Status: %s, Acc: %.3f, SA: %.3f", is_opt, a_opt, df_opt)
		    loginfo(log_str)

			publish( acc_pub_obj,   Float32Msg(a_opt) )
			publish( steer_pub_obj, Float32Msg(df_opt) )

			path_msg = mpc_path()
			path_msg.xs = x_ref
			path_msg.ys = y_ref
			path_msg.psis = psi_ref
			publish(path_pub_obj, path_msg)

			kmpc.update_current_input(df_opt, a_opt)
			res = kmpc.get_solver_results()

			mpc_path_msg = mpc_path()
			mpc_path_msg.xs = res[1] # x_mpc
			mpc_path_msg.ys = res[2] # y_mpc
			mpc_path_msg.psis = res[4] # psi_mpc	
			publish(mpc_path_pub_obj, mpc_path_msg)
		else
			publish( acc_pub_obj,   Float32Msg(-1.0) )
			publish( steer_pub_obj, Float32Msg(0.0) )		
		end
	
	    rossleep(loop_rate)
	end
end	

function start_mpc_node()
    init_node("dbw_mpc_pf")
    acc_pub   = Publisher("/control/accel", Float32Msg, queue_size=2)
    steer_pub = Publisher("/control/steer_angle", Float32Msg, queue_size=2)

    acc_enable_pub   = Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=true)
    steer_enable_pub = Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=true)


    path_pub = Publisher("target_path",  mpc_path, queue_size=2)
    mpc_path_pub = Publisher("mpc_path", mpc_path, queue_size=2)
	sub_state  = Subscriber("state_est", state_est, state_est_callback, queue_size=2)    

	publish(acc_enable_pub, UInt8Msg(2))
	publish(steer_enable_pub, UInt8Msg(1))

    pub_loop(acc_pub, steer_pub, path_pub, mpc_path_pub)    
end

if ! isinteractive()
	try 
    	start_mpc_node()
    catch x
    	print(x)
    end
end

