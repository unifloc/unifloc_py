# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 12:25:50 2019

Модуль с функциями рассчета есстественной и общей сепарацией

В данный момент реализовано:
    1. Новая методика Маркеза, скопированная с VBA

@author: oleg kobzar
"""
#импорт модулей
import sys #для подключения других частей unifloc
sys.path.append('../') #для подключения других частей unifloc
import uPVT.PVT as PVT
import math




pi = math.pi
const_g=9.81

def unf_calc_natural_separation(dtub_m, dcas_m, qliq_scm3day,
                               qg_sc_m3day, bo_m3m3, bg_m3m3,
                               sigma_o,
                               rho_osc, rho_gsc, wct, units=1):
    if qliq_scm3day == 0 or dtub_m == dcas_m:
        print('ll')
        return 1
    #water-gas surface tension coefficient (Newton/m)
    sigma_w=0.01
    a_p = pi * (dcas_m**2-dtub_m**2)/4
    q_g = bg_m3m3 * qg_sc_m3day
    q_l=bo_m3m3*qliq_scm3day*(1-wct/100) + qliq_scm3day * wct/100
    '''calculate oil density'''
    #volume fraction of liquid at no-slip conditions
    lambda_l=q_l/(q_l+q_g)
    #densities
    rho_o=rho_osc/bo_m3m3
    #todo replace water density
    rho_w=1000
    rho_l=rho_o*(1-wct/100)+rho_w*wct/100
    rho_g=rho_gsc/bg_m3m3
    #no-slip mixture density
    rho_n = rho_l * lambda_l + rho_g * (1 - lambda_l)
    #Gas sureficial velocity
    V_sg = 0.000011574 * q_g / a_p
    #Liquid sureficial velocity
    V_sl = 0.000011574 * q_l / a_p
    #----------------------
    #Liquid surface tension
    sigma_l = sigma_o * (1 - wct / 100) + sigma_w * wct / 100
    #Sureficial velocities
    v_m=V_sl+V_sg
    #Froude number
    n_fr = v_m ** 2 / (const_g * (dcas_m - dtub_m))
    #determine flow pattern
    if n_fr >= (316 * lambda_l ** 0.302) or n_fr >= (0.5 * lambda_l **( -6.738)):
        flow_pattern=2
    else:
        if n_fr <= 0.000925 * lambda_l ** -2.468:
            flow_pattern=0
        else:
            if n_fr <= 0.1 * lambda_l ** -1.452:
                flow_pattern=3
            else:
                flow_pattern=1
    #Calculate terminal gas rise velosity
    if flow_pattern == 0 or flow_pattern == 1:
        v_inf=1.53 * (const_g * sigma_l * (rho_l - rho_g) / rho_l ** 2) ** 0.25
    else:
        v_inf = 1.41 * (const_g * sigma_l * (rho_l - rho_g) / rho_l ** 2) ** 0.25
    a=-0.0093
    B=57.758
    c=34.4
    d=1.308
    ST=272
    backst=1/272
    M=V_sl/v_inf
    if M>13:
        return 0
    else:
        return ((1 + (a * B + c * M ** d) / (B + M ** d)) ** ST + M ** ST) ** backst - M
        
pintake_atm=80
q_sm3day=100
wct_perc=22
tintake_c=80
dintake_mm=100
dcasing_mm=125
gammagas=0.9
gammaoil=0.75
gammawater=1
rsb_m3m3=80
rp_m3m3=80
#оптциональные/калибровочные параметры
pb_atm=150
tres_c=120
bob_m3m3=1


def qgas_scm3day(qo_scm3day,rp_m3m3=rp_m3m3,qgfree_scm3day=0):
    return qo_scm3day*rp_m3m3+qgfree_scm3day


def out(number,string):
    space=' - '
    print(str(number),space,string)

def MF_SeparNat_d(pin_atm=q_sm3day, qliq_m3day=q_sm3day,wct_perc=wct_perc,
                  tin_c=tintake_c, dintake_mm=dintake_mm, dcas_mm=dcasing_mm,
                  gammagas=gammagas, gammaoil=gammaoil, gammawater=gammawater,
                  rsb_m3m3=rsb_m3m3,rp_m3m3=rp_m3m3,pb_atm=pb_atm,
                  tres_c=tres_c,muob_cp=1, pvtcorr='StandingBased'):
    
    pass





'''Дальше идет неактивный кусок кода, считающий коэффициент
    естесственной сепарации и сравнивая его с результатами VBA, 
    находящимися в файле SepTestData.xlsx

import matplotlib.pyplot as plt
import pandas as pd

fluid=PVT.FluidStanding(gammaoil,gammagas,gammawater,rsb_m3m3)
fluid.calc(pintake_atm,tintake_c)


data = pd.read_excel('SepTestData.xlsx')

out(dintake_mm,'dintake_mm')
out(dcasing_mm,'dcasing_mm')
out(q_sm3day,'Qliq_scm3day')
out(fluid.bo_m3m3,'bo_m3m3')
out(fluid.bg_m3m3,'bg_m3m3')
out(fluid.sigma_oil_Nm,'sigma_oil_Nm')
out(fluid.rho_oil_stkgm3,'rho_oil_stkgm3')
out(fluid.rho_gas_sckgm3,'rho_gas_sckgm3')
out(wct_perc,'wct')

def getvalues(start=1,stop=300,step=10):
    q=[]
    sep=[]
    for i in range(start,stop,step):
        q.append(i)
        sep.append(unf_calc_natural_separation(dintake_mm/1000,dcasing_mm/1000,
                                       i,
                                       qgas_scm3day(i*(1-wct_perc/100)),fluid.bo_m3m3,
                                       fluid.bg_m3m3,
                                       130/1000,fluid.rho_oil_stkgm3,
                                       fluid.rho_gas_sckgm3,wct_perc))
    return [q,sep]

result=getvalues()

plt.plot(result[0],result[1],label='py')
plt.plot(data['Q'],data['Естественная сепарация 100 мм'],label='vba')
plt.ylabel('natural sep')
plt.xlabel('qliq_scm3day')
plt.title('different sigma_o, wct=22')
plt.legend()
plt.show()
'''