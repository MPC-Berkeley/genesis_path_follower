#Library of tire models
import numpy as np

#Fiala model. Calculates nonlinear tire curve using Fiala brush model with no longitudinal force.  Inputs are the cornering stiffness per axle (N/rad). The muP and muS are slip and slip friction coefficients. alpha = slip angle (rad) Fz = normal load on that axle (N)
def fiala(C, muP, muS, alpha, Fz):
	alphaSlide = np.abs(np.arctan( 3*muP*Fz / C ))
	Fy = np.zeros(alpha.size)

	for i in range(alpha.size):

		#Use 3rd order polynomial equation when below the tire range
		 if np.abs(alpha[i]) < alphaSlide:
		 	Fy[i] = -C * np.tan(alpha[i]) + C**2 / (3 * muP * Fz) * (2 - muS / muP) * np.tan(alpha[i]) * np.abs(np.tan(alpha[i])) - C**3/(9*muP**2*Fz**2)*np.tan(alpha[i])**3*(1-(2*muS)/(3*muP)) 
		 else :
		#Use sliding force otherwise
		 	Fy[i] = -muS * Fz * np.sign(alpha[i])

	return Fy



#Coupled tire model - accounts for derating of lateral tire force when longitudinal force is applied 
def coupledTireForces(alphaF, alphaR,  FxF, FxR, vehicle):
	muF = vehicle.muF
	muR = vehicle.muR

	FzF = vehicle.FzF
	FzR = vehicle.FzR

	Cf = vehicle.Cf
	Cr = vehicle.Cr

	FyF, zetaF = _coupledTire(alphaF, FxF, FzF, muF, muF, Cf)
	FyR, zetaR = _coupledTire(alphaR, FxR, FzR, muR, muR, Cr)

	return FyF, FyR, zetaF, zetaR


#helper function, don't need to call
def _coupledTire(alpha, Fx, Fz, muS, muP, C):
	if (muP * Fz) ** 2 > Fx ** 2: 
		arg = max ((muP * Fz) **2 - Fx ** 2, 0)  #check for negative values
		zeta = np.sqrt(arg) / (muP*Fz) 	
		
	else:
		zeta = 0

	alphaSlide = np.abs( np.arctan( 3 * zeta * muP * Fz / C))

	#use fiala model, not sliding
	if abs(alpha) < alphaSlide:
		linearTerm = -C * np.tan(alpha)
		quadTerm   = C**2 * (2 - muS/muP) * abs(np.tan(alpha))*np.tan(alpha) / (3 * zeta * muP * Fz)
		cubicTerm  = - C**3 *np.tan(alpha)**3 * (1 - 2*muS / (3*muP) ) / ( 9* muP**2 * zeta**2 * Fz **2 ) 
		Fy = linearTerm + quadTerm + cubicTerm

		#Fy = -C*np.tan(alpha)+(C)**2/(3*zeta*muP*Fz)*(2-muS/muP)*abs(np.tan(alpha))*np.tan(alpha)-(C)**3/(9*(muP)**2*(zeta*Fz)**2)*(np.tan(alpha))**3*(1-2*muS/(3*muP) )

	else:
		Fy = - zeta * muS * Fz * np.sign(alpha) 

	return Fy, zeta




		
		






