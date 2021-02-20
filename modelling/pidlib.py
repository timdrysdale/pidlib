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
        self.u0 = ( -1*self.ku1*self.u1  
                - self.ku2*self.u2
                + self.ke0*self.e0 
                + self.ke1*self.e1 
                + self.ke2*self.e2); #eq12
            
        if (self.u0 > self.uMax):
            self.u0 = self.uMax; 
        if (self.u0 < self.uMin):
            self.u0 = self.uMin;
            
        return self.u0;
    
    
if __name__ == "__main__":

    pid = PID(1.0,0.0,0.0,0.02,20,-1,1)
    pid.setpoint(0.0)
    assert pid.update(0.0) == 0.0
    assert pid.update(0.5) == -0.5
    assert pid.update(1.0) == -1.0
    assert pid.update(2.0) == -1.0 #limit
    
    
    