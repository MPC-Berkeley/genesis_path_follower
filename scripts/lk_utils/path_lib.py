#This library deals with generating words

import numpy as np
from numpy import genfromtxt
import scipy.io as sio


class Path:
	def __init__(self): 
		self.roadIC = np.zeros((3,1))  #Heading(rad), Road E(m), Road N(m)
		self.posE = [0]  #east coordinates
		self.posN = [0]  #north coordinates
		self.roadPsi = [0] #heading
		self.curvature = [0] #curvature
		self.s = [0]
		self.isOpen = True #open = whether track forms loop or not
		self.referencePoint =np.zeros((3,1)) #GPS reference point
		self.refPointName = None #name of reference point
		self.friction = None  #initialize to none
		self.vMax = None

	#loads map from csv file - CRUDE
	def loadFromCSV(self, pathName):
		x = genfromtxt(pathName, delimiter =",")
		self.s = np.array(x[:,0])
		self.curvature = np.array(x[:,1])
		self.posE = np.array(x[:,2])
		self.posN = np.array(x[:,3])
		self.roadPsi = np.array(x[:,4])
		self.roadIC = np.array(x[0:3,5])

	def loadFromMAT(self, pathName):
		path = sio.loadmat(pathName, squeeze_me = True)
		self.s = path['world']['s'].sum() #No idea why you need the .sum, but otherwise doesn't work
		#self.curvature = path['world']['K'].sum()
		self.posE = path['world']['roadE'].sum()
		self.posN = path['world']['roadN'].sum()
		self.roadPsi = path['world']['roadPsi'].sum()
		self.roadIC = path['world']['road_IC'].sum()
		self.isOpen = bool(path['world']['isOpen'].sum())
		try:
			self.vMax = path['world']['vMax'].sum()
		except:
			self.vMax = None
		try:
			self.friction = path['world']['friction'].sum()
		except:
			self.friction = None
		



		






		
