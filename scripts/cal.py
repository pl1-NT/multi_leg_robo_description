#!/usr/bin/env python3

import math as m
import numpy as np
L0 = 79.929
L1 = 50.929
L2 = 66.929
L3 = 99.862
L3_1 = 84.638
L3_2 = 52.999

def kinema(t1,t2,t3):
    x = (-m.sin(t1))*(L1 +L2*m.cos(t2) +L3*m.cos(t2+t3) )
    y =  L0 + (m.cos(t1)) * (L1 +L2*m.cos(t2) +L3*m.cos(t2+t3))
    z = -(L2*m.sin(t2) + L3*m.sin(t2+t3)) 
    
    print("x:",x,"y:",y,"z:",z, sep=',')
    return x, y, z
    
    
def r_kin(x,y,z):
    t1 = m.atan(-x/(y-L0))
    
    t2_3_a = m.asin(  ( -x*x/(m.sin(t1))**2 -L1**2 +L2**2 -L3**2 -2*x*L1/m.sin(t1)  -z**2  )  /  ( 2*L3*m.sqrt( (x/m.sin(t1) + L1)**2 + z **2)     )     )
    
    #t2_3_a2 = m.asin(  ( x*x/(m.sin(t1))**2 +L1**2 -L2**2 +L3**2 +2*x*L1/m.sin(t1)  +z**2  )  /  ( 2*L3*m.sqrt( (x/m.sin(t1) + L1)**2 + z **2)     )     )
    
    t_a = m.asin( -(x/m.sin(t1) + L1)/ m.sqrt( (x/m.sin(t1) + L1)**2 + z **2) )
    
    t2_3 = t2_3_a - t_a
    print("t2_3_a:",t2_3_a)
    
    t2 =  m.asin((z -L3*m.sin(t2_3)) / L2) 
    
    t23 = m.asin((z-L2*m.sin(t2))/L3)
    
    t3 = t23 - t2
    
    print("t1:",t1,"t2:",-t2,"t3:",-t3)
    
    
if __name__ == '__main__':
    x,y,z = kinema(0.1, -0.5, 1.9)
    r_kin(0.2524326162744459 ,207.3209922455048 ,-66.08459628453034)
        
       
