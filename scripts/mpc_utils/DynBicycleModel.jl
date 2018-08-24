__precompile__()

module DynBicycleModel

	#using ForwardDiff

	function f_dyn_bicycle_model(z::Vector)
		# Genesis Parameters from HCE:
		lf = 1.5213  			# m  	(CoG to front axle)
		lr = 1.4987  			# m  	(CoG to rear axle)
		d  = 0.945	 			# m  	(half-width, currently unused)
		m  = 2303.1   			# kg 	(vehicle mass)
		Iz  = 5520.1			# kg*m2 (vehicle inertia)
		C_alpha_f = 7.6419e4*2    # N/rad	(front tire cornering stiffness)
		C_alpha_r = 13.4851e4*2	# N/rad	(rear tire cornering stiffness)
		
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
		vx_dot  = acc - (Fyf*sin(df))/m + wz*vy
		vy_dot  = (Fyf*cos(df) + Fyr)/m - wz*vx 
		wz_dot  = (lf*Fyf*cos(df) - lr*Fyr)/Iz
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

	#function jacobian_f_dyn_bicycle_model(z)
	#	return ForwardDiff.jacobian(f_dyn_bicycle_model, z)
	#end
end
