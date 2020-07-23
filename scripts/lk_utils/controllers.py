import numpy as np
import tiremodel_lib as tm
import vehicle_lib
#import path_lib
import math



#Controller from Nitin Kapania's PhD thesis - lookahead with augmented sideslip for
#steering feedback, longitudinal is simple PID feedback control
class LaneKeepingController():
    def __init__(self, path, vehicle, profile):
        self.path = path
        self.vehicle = vehicle
        self.profile = profile
        self.xLA = 7.8    #lookahead distance, meters
        self.kLK = 0.095  #proportional gain , rad / meter
        self.kSpeed = 6000.0 #Speed proportional gain - N / (m/s)
        self.alphaFlim = 7.0 * np.pi / 180 #steering limits for feedforward controller
        self.alphaRlim = 5.0 * np.pi / 180 #steering limits for feedforward controller
        
        #Initialize force lookup tables for feedforward
        numTableValues = 250

        #values where car is sliding
        alphaFslide = np.abs(np.arctan(3*vehicle.muF*vehicle.m*vehicle.b/vehicle.L*vehicle.g/vehicle.Cf)) 
        alphaRslide = np.abs(np.arctan(3*vehicle.muR*vehicle.m*vehicle.a/vehicle.L*vehicle.g/vehicle.Cr))

        alphaFtable = np.linspace(-alphaFslide, alphaFslide, numTableValues)
        alphaRtable = np.linspace(-alphaRslide, alphaRslide, numTableValues) # vector of rear alpha (rad)
        
        FyFtable = tm.fiala(vehicle.Cf, vehicle.muF, vehicle.muF, alphaFtable, vehicle.FzF)
        FyRtable = tm.fiala(vehicle.Cr, vehicle.muR, vehicle.muR, alphaRtable, vehicle.FzR)

        #flip arrays so Fy is increasing - important for numpy interp!!
        self.alphaFtable = np.flip(alphaFtable, 0)
        self.alphaRtable = np.flip(alphaRtable, 0)
        self.FyFtable = np.flip(FyFtable, 0) 
        self.FyRtable = np.flip(FyRtable, 0)
	#self.cnt = 0


    def updateInput(self, localState, controlInput):
        delta, deltaFFW, deltaFB, K = _lanekeeping(self, localState)
        Fx, UxDes, FxFFW, FxFB = _speedTracking(self, localState)
        controlInput.update(delta, Fx)
        auxVars = {'K': K , 'UxDes': UxDes}

        return auxVars


class OpenLoopControl():
    def __init__(self, vehicle, delta = 2 * np.pi / 180, Fx = 100.):
        self.delta = delta
        self.Fx = Fx
            

    #Note, Local state not needed for open loop control, no feedback!    
    def updateInput(self, localState, controlInput):
        
        delta = self.delta
        Fx = self.Fx
            
        #Curvature is 0 for open loop control - no path to track 
        auxVars = {'K': 0., 'UxDes': 0.}
        controlInput.update(delta, Fx)

        return auxVars


class ControlInput:
    def __init__(self):
        self.delta = 0.0
        self.Fx = 0.0

    def update(self, delta, Fx):
        self.delta = delta
        self.Fx = Fx



def _force2alpha(forceTable, alphaTable, Fdes):
        if Fdes > max(forceTable):
             Fdes = max(forceTable) - 1

        elif Fdes < min(forceTable):
             Fdes = min(forceTable) + 1

        #note - need to slice to rank 0 for np to work
        #note - x values must be increasing in numpy interp!!!
        alpha = np.interp(Fdes, forceTable ,alphaTable)
        

        return alpha


def _lanekeeping(sim,localState):
    
    #note - interp requires rank 0 arrays
    sTable = sim.path.s
    kTable = sim.path.curvature

    K = np.interp(localState.s, sTable, kTable) #run interp every time - this is slow, but we may be able to get away with
    #if(abs(K)>0.01):
    #	print(str(sim.cnt)+'] K: '+str(round(K,3))+' S: '+str(localState.s))
    #sim.cnt = sim.cnt + 1 
    deltaFFW, betaFFW, FyFdes, FyRdes, alphaFdes, alphaRdes = _getDeltaFFW(sim, localState, K)
    deltaFB = _getDeltaFB(sim, localState, betaFFW)
    delta = deltaFFW + deltaFB
    return delta, deltaFFW, deltaFB, K


def _speedTracking(sim, localState):

    #note - interp requires rank 0 arrays
    AxTable = sim.profile.Ax
    UxTable = sim.profile.Ux
    sTable = sim.profile.s
    m = sim.vehicle.m
    fdrag = sim.vehicle.dragCoeff
    frr = sim.vehicle.rollResistance

    s = localState.s
    Ux = localState.Ux

    AxDes = np.interp(s, sTable, AxTable) #run interp every time - this is slow, but we may be able to get away with
    UxDes = np.interp(s, sTable, UxTable) #run interp every time - this is slow, but we may be able to get away with


    FxFFW = m*AxDes + np.sign(Ux)*fdrag*Ux ** 2 + frr*np.sign(Ux) # Feedforward
    FxFB = -sim.kSpeed*(Ux - UxDes) # Feedback
    FxCommand = FxFFW + FxFB
    return FxCommand, UxDes, FxFFW, FxFB


def _getDeltaFB(sim, localState, betaFFW):
    kLK = sim.kLK
    xLA = sim.xLA
    e = localState.e
    deltaPsi = localState.deltaPsi

    deltaFB = -kLK * (e + xLA * np.sin(deltaPsi + betaFFW))
    return deltaFB

def _getDeltaFFW(sim, localState, K):
    a = sim.vehicle.a
    b = sim.vehicle.b
    L = sim.vehicle.L
    m = sim.vehicle.m
    Ux = localState.Ux


    FyFdes = b / L * m * Ux**2 * K
    FyRdes = a / b * FyFdes

    alphaFdes = _force2alpha(sim.FyFtable, sim.alphaFtable, FyFdes)
    alphaRdes = _force2alpha(sim.FyRtable, sim.alphaRtable, FyRdes)

    betaFFW = alphaRdes + b * K 
    deltaFFW = K * L + alphaRdes - alphaFdes

    return deltaFFW, betaFFW, FyFdes, FyRdes, alphaFdes, alphaRdes        







