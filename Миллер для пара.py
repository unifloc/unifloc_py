# -*- coding: utf-8 -*-
"""
Created on Tue Aug 14 16:41:53 2018

@author: Артём
"""
"""
расчет давления на входе в зависимости от расхода диаметра и давления на выходе

"""
import numpy as np
import matplotlib.pyplot as plt
d0_mm=5  # диаметр отверстия
p2_atm=10 # давление за отверстием
w_kghr=5
w_kghr_a = np.arange(0.5,10,0.1)
p2_atm_a = np.arange(5,13,2)
def p1_atm(d0_mm, w_kghr, p2_atm):
    d1_mm=100 # диаметр трубы
    C0 =0.8 # коэффициент разряда штуцера
    Y1=0.89487 # коэффициент расширения пара
    vfg_m3kg=57.8 # удельный объем пара
    ro_kgm3=1/vfg_m3kg
    for p2_atm in p2_atm_a:
        for w_kghr in w_kghr_a:
            beta=d0_mm/d1_mm
            w_kgsec=w_kghr/3600
            f_kPa=w_kgsec**2*(1-beta**4)*10**10/(12.337*C0**2*Y1**2*d0_mm**4*ro_kgm3)
            p1_atm=f_kPa/100+p2_atm
            return p1_atm
print(p1_atm(d0_mm, w_kghr, p2_atm))


"""
расчет давления на входе в зависимости от расхода при разных давления на выходе

"""
d0_mm=10  # диаметр отверстия
p2_atm=5 # давление за отверстием
w_kghr=5
w_kghr_a = np.arange(0,20,1)
def p1_atm(d0_mm, w_kghr, p2_atm):
    d1_mm=100 # диаметр трубы
    C0 =0.8 # коэффициент разряда штуцера
    Y1=0.89487 # коэффициент расширения пара
    vfg_m3kg=57.8 # удельный объем пара
    ro_kgm3=1/vfg_m3kg
    beta=d0_mm/d1_mm
    w_kgsec=w_kghr/3600
    f_kPa=w_kgsec**2*(1-beta**4)*10**10/(12.337*C0**2*Y1**2*d0_mm**4*ro_kgm3)
    p1_atm=p2_atm+f_kPa/100
    return p1_atm
print(p1_atm(d0_mm, w_kghr, p2_atm))
q1=np.array([])
q2=np.array([])
q3=np.array([])
q4=np.array([])
q5=np.array([])
for w in w_kghr_a:
    p1=p1_atm(d0_mm, w, 5)
    q1=np.append(q1,p1) 
    p1=p1_atm(d0_mm, w, 7)
    q2=np.append(q2,p1)
    p1=p1_atm(d0_mm, w, 9)
    q3=np.append(q3,p1)
    p1=p1_atm(d0_mm, w, 11)
    q4=np.append(q4,p1)
    p1=p1_atm(d0_mm, w, 13)
    q5=np.append(q5,p1)
plt.plot(w_kghr_a,q1, label='Рвых=5 атм')
plt.plot(w_kghr_a,q2, label='Рвых=7 атм')
plt.plot(w_kghr_a,q3, label='Рвых=9 атм')
plt.plot(w_kghr_a,q4, label='Рвых=11 атм')
plt.plot(w_kghr_a,q5, label='Рвых=13 атм')
plt.title('Давление на входе в зависимости от расхода и давления на выходе')
plt.xlabel('Расход пара, кг/час')
plt.ylabel('Давление на входе, атм')
plt.legend()
plt.show() 


"""
расчет давления на выходе в зависимости от давления на входе при разных расходах

"""

d0_mm=5  # диаметр отверстия
w_kghr=5
p1_atm=15 # давление за отверстием
p1_atm_a = np.arange(20,25,1)
def p2_atm(d0_mm, w_kghr, p1_atm):
    d1_mm=100 # диаметр трубы
    C0 =0.8 # коэффициент разряда штуцера
    Y1=0.89487 # коэффициент расширения пара
    vfg_m3kg=57.8 # удельный объем пара
    ro_kgm3=1/vfg_m3kg
    beta=d0_mm/d1_mm
    w_kgsec=w_kghr/3600
    f_kPa=w_kgsec**2*(1-beta**4)*10**10/(12.337*C0**2*Y1**2*d0_mm**4*ro_kgm3)
    p2_atm=p1_atm-f_kPa/100
    return p2_atm
print(p2_atm(d0_mm, w_kghr, p1_atm))
q1=np.array([])
q2=np.array([])
q3=np.array([])
q4=np.array([])
q5=np.array([])
for p1 in p1_atm_a:
    p2=p2_atm(d0_mm, 0, p1)
    q1=np.append(q1,p2) 
    p2=p2_atm(d0_mm, 3, p1)
    q2=np.append(q2,p2)
    p2=p2_atm(d0_mm, 7, p1)
    q3=np.append(q3,p2)
    p2=p2_atm(d0_mm, 11, p1)
    q4=np.append(q4,p2)
    p2=p2_atm(d0_mm, 13, p1)
    q5=np.append(q5,p2)
plt.plot(p1_atm_a,q1, label='0 кг/час')
plt.plot(p1_atm_a,q2, label='3 кг/час')
plt.plot(p1_atm_a,q3, label='7 кг/час')
plt.plot(p1_atm_a,q4, label='11 кг/час')
plt.plot(p1_atm_a,q5, label='13 кг/час')
plt.title('Зависимость давления на выходе от давления на входе и расхода')
plt.xlabel('Давление на входе, атм')
plt.ylabel('Давление на выходе, атм')
plt.legend()
plt.show() 

"""
расчет расхода пара от перепада давления

"""
d0_mm=5  # диаметр отверстия
f_atm=5
f_atm_a = np.arange(0,10,0.5)
def w_kghr(d0_mm, f_atm):
    d1_mm=100 # диаметр трубы
    C0 =0.8 # коэффициент разряда штуцера
    Y1=0.89487 # коэффициент расширения пара
    vfg_m3kg=57.8 # удельный объем пара
    ro_kgm3=1/vfg_m3kg
    beta=d0_mm/d1_mm
    f_kPa=f_atm*100
    w_kgsec=3.512407*10**(-5)*C0*Y1*d0_mm**2*(f_kPa*ro_kgm3)**0.5/(1-beta**4)**0.5
    w_kghr=w_kgsec*3600
    return w_kghr
q=np.array([])
for f in f_atm_a:
    w=w_kghr(d0_mm, f)
    q=np.append(q,w)     
plt.plot(f_atm_a,q)
plt.title('Расход пара от перепада давления за и перед штуцером')
plt.xlabel('Перепад давления, атм')
plt.ylabel('Расход пара, кг/час')
plt.show()