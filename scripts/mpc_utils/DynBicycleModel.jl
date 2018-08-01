__precompile__()

module DynBicycleModel

	using ForwardDiff

	function f_dyn_bicycle_model(z::Vector)
		# Azera Params taken from:
		# https://github.com/MPC-Car/Controller/blob/master/LearningController/RaceCar_ILMPC_4_NewFormulation/src/init/genHeader_data_vehicle.m
		lf = 1.152  			# m  	(CoG to front axle)
		lr = 1.693  			# m  	(CoG to rear axle)
		d  = 0.8125 			# m  	(half-width, currently unused)
		m  = 1840   			# kg 	(vehicle mass)
		Iz  = 3477				# kg*m2 (vehicle inertia)
		C_alpha_f = 4.0703e4    # N 	(front tire cornering stiffness)
		C_alpha_r = 6.4495e4	# N 	(rear tire cornering stiffness)
		
		X   = z[1]
		Y   = z[2] 
		psi = z[3]
		vx  = z[4]
		vy  = z[5]
		wz  = z[6]

		acc  = z[7]
		df   = z[8]
		
		# Compute tire slip angle
		alpha_f = 0.0
		alpha_r = 0.0
		if abs(vx) > 2.0
			alpha_f = df - atan2( vy+lf*wz, vx )
			alpha_r = -atan2(vy-lr*wz , vx)        		
		end

		# Compute lateral force at front and rear tire (linear model)
		Fyf = C_alpha_f * alpha_f
		Fyr = C_alpha_r * alpha_r

		# Propagate the vehicle dynamics deltaT seconds ahead.			
		#vx_dot  = acc - 1/m*Fyf*sin(df) + wz*vy
		#vy_dot  = 1.0/m*(Fyf*cos(df) + Fyr) - wz*vx 
		#wz_dot  = 1.0/Iz*(lf*Fyf*cos(df) - lr*Fyr)

		vx_dot  = acc - 1/m*Fyf*df + wz*vy
		vy_dot  = 1.0/m*(Fyf + Fyr) - wz*vx 
		wz_dot  = 1.0/Iz*(lf*Fyf - lr*Fyr)  
		
		psi_dot = wz
		X_dot   = vx*cos(psi) - vy*sin(psi)
		Y_dot   = vx*sin(psi) + vy*cos(psi)


		z_dot = Vector(6)
		z_dot[1] = X_dot
		z_dot[2] = Y_dot
		z_dot[3] = psi_dot
		z_dot[4] = vx_dot
		z_dot[5] = vy_dot
		z_dot[6] = wz_dot
		return z_dot
	end

	function jacobian_f_dyn_bicycle_model(z)
		return ForwardDiff.jacobian(f_dyn_bicycle_model, z)
	end

	#= Testing Code, unused:
	function finite_differences(f,z, eps=1e-6)
		J = zeros(6,8)

		for i = 1:8
			z_hi = copy(z)
			z_low = copy(z)

			z_hi[i] = z[i] + eps
			z_low[i] = z[i] - eps

			fhi = f(z_hi)
			flow = f(z_low)

			J[:,i] = (fhi - flow)/(2.0 * eps)				
		end

		return J
	end

	j = z -> ForwardDiff.jacobian(f_dyn_bicycle_model,z);
	j2 = z -> finite_differences(f_dyn_bicycle_model,z);
	diff_j = z-> norm(convert(Array{Float64,2},j(z)) - j2(z))

	diff_in_jac = 0.0
	for i = 1:1000
		z_in = 100*rand(8)
		#z_in[7] = 1.0

		#println(j(z_in))
		#println(j2(z_in))
		diff_in_jac = diff_in_jac + 1/1000* diff_j(z_in)
	end
	println(diff_in_jac)
	=#
end
