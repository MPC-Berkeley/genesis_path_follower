import numpy as np
import tiremodel_lib as tm
#Defines a vehicle class, with full parameters

#Defaults to shelley variables
class Vehicle:
	def __init__(self, vehicleName = "shelley"): 

		if vehicleName is "shelley":
			
			self.a = 1.0441 #CG to front wheelbase [m]
			self.b = 1.4248 #CG to rear wheelbase [m] 
			self.m = 1512.4 #vehicle mass (kg)
			self.Cf = 160000.0 #vehicle cornering stiffness (N)
			self.Cr = 180000.0 #vehicle cornering stiffness (N)
			self.Iz  = 2.25E3  #vehicle inertia (kg  m^2)
			self.muF = 0.97     #front friction coeff
			self.muR = 1.02    #rear friction coeff
			self.g = 9.81      #m/s^2, accel due to gravity
			self.L = self.a + self.b #total vehicle length, m
			self.FzF = self.m*self.b*self.g/self.L   #Maximum force on front vehicles
			self.FzR = self.m*self.a*self.g/self.L   #Maximium force on rear vehicles
			self.D = 0.3638 #Drag coefficient
			self.h = 0.75   #Distance from the ground
			self.brakeTimeDelay = 0.25 #Seconds
			self.rollResistance = 255.0 #Newtons
			self.maxSpeed = 5.0
			self.powerLimit = 160000.0 #Watts
			self.dragCoeff = 0.3638 #N / (m/s)^2
			self.deltaLim = 27. * np.pi / 180  #Steering limit, radians

		##NOTE: Need to fill in with genesis parameters!!!!!!
		elif vehicleName is "genesis":

			self.a = 1.0441 #CG to front wheelbase [m]
			self.b = 1.4248 #CG to rear wheelbase [m] 
			self.m = 1512.4 #vehicle mass (kg)
			self.Cf = 160000.0 #vehicle cornering stiffness (N)
			self.Cr = 180000.0 #vehicle cornering stiffness (N)
			self.Iz  = 2.25E3  #vehicle inertia (kg  m^2)
			self.muF = 0.97     #front friction coeff
			self.muR = 1.02    #rear friction coeff
			self.g = 9.81      #m/s^2, accel due to gravity
			self.L = self.a + self.b #total vehicle length, m
			self.FzF = self.m*self.b*self.g/self.L   #Maximum force on front vehicles
			self.FzR = self.m*self.a*self.g/self.L   #Maximium force on rear vehicles
			self.D = 0.3638 #Drag coefficient
			self.h = 0.75   #Distance from the ground
			self.brakeTimeDelay = 0.25 #Seconds
			self.rollResistance = 255.0 #Newtons
			self.maxSpeed = 10
			self.powerLimit = 160000.0 #Watts
			self.dragCoeff = 0.3638 #N / (m/s)^2
			self.deltaLim = 27. * np.pi / 180  #Steering limit, radians

