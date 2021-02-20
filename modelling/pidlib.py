#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 20 17:41:42 2021

@author: tim
"""

class PID:
    def __init__(self, Kp, Ki, Kd, Ts, N, uMin, uMax):
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.N = N
        self.uMin = uMin
        self.uMax = uMax
        self.reset()
        
    def reset(self):
        
        self.e0 = 0;
        self.e1 = 0;
        self.e2 = 0;
        self.u0 = 0;
        self.u1 = 0;
        self.u2 = 0;
        
        self.a0 = (1+self.N*self.Ts);
        self.a1 = -(2 + self.N*self.Ts);
        self.a2 = 1;
        self.b0 = self.Kp*(1+self.N*self.Ts) + self.Ki*self.Ts*(1+self.N*self.Ts) + self.Kd*self.N;
        self.b1 = -(self.Kp*(2+self.N*self.Ts) + self.Ki*self.Ts + 2*self.Kd*self.N);
        self.b2 = self.Kp + self.Kd*self.N;
        self.ku1 = self.a1/self.a0;
        self.ku2 = self.a2/self.a0;
        self.ke0 = self.b0/self.a0;
        self.ke1 = self.b1/self.a0;
        self.ke2 = self.b2/self.a0;
       
    def setpoint(self,r):
        self.r = r
        
    def update(self,y):
        self.e2=self.e1;
        self.e1=self.e0;
        self.u2=self.u1;
        self.u1=self.u0;
            
        self.e0=self.r-y; #compute new error
        
        if (self.e0 * self.e1 <= 0):
            self.u1 = 0;
            self.u2 = 0;
        
        self.u0 = ( -1*self.ku1*self.u1  
                - self.ku2*self.u2
                + self.ke0*self.e0 
                + self.ke1*self.e1 
                + self.ke2*self.e2); #eq12
            
        if (self.u0 > self.uMax):
            self.u0 = self.uMax; 
        if (self.u0 < self.uMin):
            self.u0 = self.uMin;
        
        #print(self.u1,self.u2,self.e0,self.e1,self.e2)
        return self.u0;
    
    
if __name__ == "__main__":
    
    import math
    import numpy as np
    
    # PROPORTIONAL TEST
    Ts = 0.02
    N = 20
    
    p = PID(1.0,0.0,0.0,Ts,N,-1,1)
    p.setpoint(0.0)
    assert p.update(0.0) == 0.0
    assert p.update(0.5) == -0.5
    assert p.update(1.0) == -1.0
    assert p.update(2.0) == -1.0 #limit
    
    # INTEGRAL TEST
    Ki = 10
    i = PID(0.0,Ki,0.0,0.02,20,-1,1)
    i.setpoint(0.0)
    err = 0.1
    for step in range(1,51):
        expected = -1.0 * step * err  *Ts * Ki
        actual =i.update(err)
        assert math.isclose(expected, actual)
    
    #now at limit
    expected = -1.0 * step * err  *Ts * Ki
    for step in range(1,50):
        actual = i.update(err)
        assert math.isclose(expected, actual)
     
    # check that integral term is zero'd after reaching setpoint    
    actual = i.update(0.0) #grace of one step due to error history
    actual = i.update(0.0)
    assert math.isclose(0, actual)
    
    #repeat, this time pass zero and check no windup
    i = PID(0.0,Ki,0.0,0.02,20,-1,1)
    i.setpoint(0.0)
    err = 0.1
    for step in range(1,51):
        expected = -1.0 * step * err  *Ts * Ki
        actual =i.update(err)
        assert math.isclose(expected, actual)
    
    #now at limit
    expected = -1.0 * step * err  *Ts * Ki
    for step in range(1,50):
        actual = i.update(err)
        assert math.isclose(expected, actual)

    # check that integral term is zero'd after reaching setpoint    
    actual = i.update(-0.1)
    assert (actual > 0 and actual <= 2 * err  *Ts * Ki)
    
    # DERIVATIVE TEST
    Kd = 1.0 
    Ts = 0.02
    N = 20
    d = PID(0.0,0.0,Kd,Ts,N,-1,1)
    setpoint = 0.0
    d.setpoint(setpoint)
    d.update(setpoint)
    
    deltaPlant = 0.01
    plant = setpoint
    deltaY = 0
    lastDeltaY = 0
    lastY = 0
    y = 0
    for step in range(1,90):
        plant = plant + deltaPlant
        lastdeltaY = deltaY
        lastY = y
        y = d.update(plant)
        lastDeltaY = deltaY
        deltaY = y - lastY
        if (step > 1):
            assert (y < lastY) #monotonic behaviour
        if (step > 2):    
            print(deltaY, lastDeltaY)
            assert (deltaY > lastDeltaY) #asymptotic approaching steady state
       
    expected = -1 * deltaPlant/ Ts
    assert math.isclose(expected, y)
    
    
    

        