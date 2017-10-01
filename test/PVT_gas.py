# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 18:14:04 2017

@author: rnt

функции расчета свойств углеводородных газов
"""
import math
import numpy as np

def dranchuk_z_factor(T, p, T_cr, p_cr):
    """
    dranchuk's z-factor based on the newton's method
    """
    A = np.array([0.3265, -1.0700, -0.5339, 0.01569, -0.05165, 0.5475, -0.7361, 
                 0.1844, 0.1056, 0.6134, 0.7210])
    p_r=p/p_cr  #reduced pressure
    T_r=T/T_cr  #reduced temperature
    if not((0.0<=p_r<30.0 and 1.0<T_r<=3.0) or (p_r<1.0 and 0.7<T_r<=1.0)): #in the article 0.2<=p_r<30 is recommended
        print ('Warning! Temperature and pressure are out of range Dranchuck corr, p_r=', p_r, 'T_r=', T_r)
    C1=A[0]+A[1]/T_r+A[2]/T_r**3+A[3]/T_r**4+A[4]/T_r**5
    C2=A[5]+A[6]/T_r+A[7]/T_r**2
    C3=A[8]*(A[6]/T_r+A[7]/T_r**2)
    def dranchuk_eq(z):
        rho_r=0.27*p_r/(z*T_r) #reduced density
        return -z+1+C1*rho_r+C2*rho_r**2-C3*rho_r**5+A[9]*(1+A[10]*rho_r**2)*(rho_r**2/T_r**3)*np.exp(-A[10]*rho_r**2)
        
    def dranhuck_eq_deriv(z):
        D=0.27*p_r/z
        exp_cf=np.exp((-A[10]*D**2)/T_r**2)
        K=2*A[9]*A[10]*(1+A[10]*D**2/T_r**2)*D**4*exp_cf/(T_r**7)
        C1=D*(A[0]+A[1]/T_r+A[2]/T_r**3+A[3]/T_r**4+A[4]/T_r**5)/(z*T_r)
        C2=(2*D**2)*(A[5]+A[6]/T_r+A[7]/T_r**2)/(z*T_r**2)
        C3=(5*A[8]*D**5)*(A[6]/T_r+A[7]/T_r**2)/(z*T_r**5)
        C4=A[9]*D**2*exp_cf*(2+4*A[10]*D**2/T_r**2)/(z*T_r**5)
        return -1-C1-C2+C3-C4+K/z
        
    solution=newton(dranchuk_eq, 1, dranhuck_eq_deriv)
    return solution
    

#partial derivative of z-factor by absolute temperature
def dranchuk_z_part_dev_T(z, T, p, T_cr, p_cr):
    """
    #z is z-factor
    #T - gas temperature (kelvin)
    #p - gas pressure (any units)
    #T_cr - critical temperature (kelvin)
    #p_cr - critical pressure (any units)
    """
    A = np.array([0.3265, -1.0700, -0.5339, 0.01569, -0.05165, 0.5475, -0.7361, 
                 0.1844, 0.1056, 0.6134, 0.7210])
    p_r=p/p_cr  #reduced pressure
    T_r=T/T_cr  #reduced temperature
    D=0.27*p_r/z
    exp_cf=np.exp((-A[10]*D**2)/T_r**2)
    K=2*A[9]*A[10]*(1+A[10]*D**2/T_r**2)*D**4*exp_cf/(T_r**7)
    
    C1=D*(A[0]+A[1]/T_r+A[2]/T_r**3+A[3]/T_r**4+A[4]/T_r**5)/(z*T_r)
    C2=(2*D**2)*(A[5]+A[6]/T_r+A[7]/T_r**2)/(z*T_r**2)
    C3=(5*A[8]*D**5)*(A[6]/T_r+A[7]/T_r**2)/(z*T_r**5)
    C4=A[9]*D**2*exp_cf*(2+4*A[10]*D**2/T_r**2)/(z*T_r**5)
    
    denominator=-1-C1-C2+C3-C4+K/z
    
    L1=D*(A[0]+2*A[1]/T_r+4*A[2]/T_r**3+5*A[3]/T_r**4+6*A[4]/T_r**5)/(T*T_r)
    L2=(D**2)*(2*A[5]+3*A[6]/T_r+4*A[7]/T_r**2)/(T*T_r**2)
    L3=(A[8]*D**5)*(6*A[6]/T_r+7*A[7]/T_r**2)/(T*T_r**5)
    L4=A[9]*D**2*exp_cf*(5+7*A[10]*D**2/T_r**2)/(T*T_r**5)
    
    numerator=-L1-L2+L3-L4+K/T
    
    return -numerator/denominator

def dPdz_annulus(P, T, Z, M):
    R=8.314
    g = 9.81
    return P*M*g/(Z*R*T)