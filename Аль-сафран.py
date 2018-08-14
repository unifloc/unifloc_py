# -*- coding: utf-8 -*-
"""
Created on Tue Aug 14 16:15:47 2018

@author: Артём
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
""" 
расчет расхода в зависимости от давления на выходе, при разных давлениях на входе

"""
p1_atm_a= np.arange(10,20,2)
for p1_atm in p1_atm_a:
    p2_a=np.arange(1,p1_atm,0.1)
    def wi(p1_atm, p2_atm):
        k=1.3 # показатель адиабаты
        Cd=0.75 # дисчарч коэф для штуцера
        g_c=9.8 # ускорение свободного падения
        p1_pa=p1_atm*10**5 # давление до штуцера Па
        p2_pa=p2_atm*10**5 # давление после штуцера Па
        Vg1=57.8 # удельный объем пара м3/кг
        Vl=0.001 # удельный объем жидкости (в данном случае воды) м3/кг
        CL=4.176 # удельная теплоемкость жидкости кДж/кг/К
        Cvg=1.539 # удельная теплоемкость пара кДж/кг/К
        d2=0.005 # диаметр штуцера м
        xg=1 #  массовая доля пара
        R=(1+xg*(Vg1/Vl-1))**0.5*(1+0.6/2.718**(5*xg))
        alfa=R*(1-xg)*Vl/(xg*Vg1)
        A2=3.14*d2**2/4
        n=(xg*k*Cvg+(1-xg)*CL)/(xg*Cvg+(1-xg)*CL)
        def y(x):
            y=(alfa*(1-x)+n/(n-1))/(n/(n-1)+n/2*(1+alfa*x**(1/n))**2)-x**(1-1/n)
            return(y)
        rc = opt.fsolve(y, 0.5)
        r=p2_pa/p1_pa
        if rc >= r: pr=rc
        else: pr=r
        Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
        Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
        wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
        return wi
    q=np.array([])
    for p2 in p2_a:
        p1=wi(p1_atm, p2)
        q=np.append(q,p1)
    plt.plot(p2_a,q,label='Рвх=%1.1f'%p1_atm)
plt.title('Расход пара от давления на выходе')
plt.ylabel('Расход пара, кг/час')
plt.xlabel('Давление на выходе, атм')
plt.legend()
plt.show()


""" 
расчет давления на входе в зависимости от расхода, при разных давлениях на выходе

"""
p2_atm_a= np.arange(5,15,2)
for p2_atm in p2_atm_a:
    def wi(p1_atm): 
        k=1.3 # показатель адиабаты
        Cd=0.75 # дисчарч коэф для штуцера
        g_c=9.8 # ускорение свободного падения
        p1_pa=p1_atm*10**5 #
        p2_pa=p2_atm*10**5 #
        Vg1=57.8 # удельный объем пара
        Vl=0.001 # удельный объем жидкости (в данном случае воды)
        CL=4.176 # удельная теплоемкость жидкости кДж/кг/К
        Cvg=1.539 # удельная теплоемкость пара кДж/кг/К
        d2=0.005 # диаметр штуцера
        xg=1 #  массовая доля пара
        R=(1+xg*(Vg1/Vl-1))**0.5*(1+0.6/2.718**(5*xg))
        alfa=R*(1-xg)*Vl/(xg*Vg1)
        A2=3.14*d2**2/4
        n=(xg*k*Cvg+(1-xg)*CL)/(xg*Cvg+(1-xg)*CL)
        def y(x):
            y=(alfa*(1-x)+n/(n-1))/(n/(n-1)+n/2*(1+alfa*x**(1/n))**2)-x**(1-1/n)
            return(y)
        rc = opt.fsolve(y, 0.5)
        r=p2_pa/p1_pa
        if rc >= r: pr=rc
        else: pr=r
        Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
        Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
        wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
        return wi
    i_a=np.arange(0,300,10)
    for i in i_a:
        def w2(p1_atm):
            return wi(p1_atm)-i
        def p1_atm(w2):
            p1_atm=opt.fsolve(w2,p2_atm)
            return p1_atm
    q=np.array([])
    for i in i_a:
        p1=p1_atm(w2)
        q=np.append(q,p1)
    plt.plot(i_a,q,label='Рвых=%1.1f'%p2_atm)
plt.title('Давление на входе в зависимости от расхода и давления на выходе')
plt.xlabel('Расход пара, кг/час')
plt.ylabel('Давление на входе, атм')   
plt.legend()
plt.show() 


""" 
расчет давления на выходе в зависимости от расхода, при разных давлениях на входе

"""
p1_atm_a= np.arange(10,20,2)
for p1_atm in p1_atm_a:
    def wi(p2_atm):
        k=1.3 # показатель адиабаты
        Cd=0.75 # дисчарч коэф для штуцера
        g_c=9.8 # ускорение свободного падения
        p1_pa=p1_atm*10**5 #
        p2_pa=p2_atm*10**5 #
        Vg1=57.8 # удельный объем пара
        Vl=0.001 # удельный объем жидкости (в данном случае воды)
        CL=4.176 # удельная теплоемкость жидкости кДж/кг/К
        Cvg=1.539 # удельная теплоемкость пара кДж/кг/К
        d2=0.005 # диаметр штуцера
        xg=1 #  массовая доля пара
        R=(1+xg*(Vg1/Vl-1))**0.5*(1+0.6/2.718**(5*xg))
        alfa=R*(1-xg)*Vl/(xg*Vg1)
        A2=3.14*d2**2/4
        n=(xg*k*Cvg+(1-xg)*CL)/(xg*Cvg+(1-xg)*CL)
        def y(x):
            y=(alfa*(1-x)+n/(n-1))/(n/(n-1)+n/2*(1+alfa*x**(1/n))**2)-x**(1-1/n)
            return(y)
        rc = opt.fsolve(y, 0.5)
        r=p2_pa/p1_pa
        if rc >= r: pr=rc
        else: pr=r
        Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
        Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
        wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
        return wi
    i_a=np.arange(0,150,10)
    for i in i_a:
        def w2(p2_atm):
            return wi(p2_atm)-i
        def p2_atm(w2):
            p2_atm=opt.fsolve(w2,p1_atm-0.00001)
            return p2_atm
    q=np.array([])
    for i in i_a:
        p2=p2_atm(w2)
        q=np.append(q,p2)
    plt.plot(i_a,q,label='Рвх=%1.1f'%p1_atm)
plt.title('Давление на выходе в зависимости от расхода и давления на входе')
plt.xlabel('Расход пара, кг/час')
plt.ylabel('Давление на выходе, атм')   
plt.legend()
plt.show()  

""" 
расчет давления на выходе в зависимости от давления на входе, при разных расходах

"""
i_a=np.arange(0,200,50)
for i in i_a:
    def wi(p2_atm):
        k=1.3 # показатель адиабаты
        Cd=0.75 # дисчарч коэф для штуцера
        g_c=9.8 # ускорение свободного падения
        p1_pa=p1_atm*10**5 #
        p2_pa=p2_atm*10**5 #
        Vg1=57.8 # удельный объем пара
        Vl=0.001 # удельный объем жидкости (в данном случае воды)
        CL=4.176 # удельная теплоемкость жидкости кДж/кг/К
        Cvg=1.539 # удельная теплоемкость пара кДж/кг/К
        d2=0.005 # диаметр штуцера
        xg=1 #  массовая доля пара
        R=(1+xg*(Vg1/Vl-1))**0.5*(1+0.6/2.718**(5*xg))
        alfa=R*(1-xg)*Vl/(xg*Vg1)
        A2=3.14*d2**2/4
        n=(xg*k*Cvg+(1-xg)*CL)/(xg*Cvg+(1-xg)*CL)
        def y(x):
            y=(alfa*(1-x)+n/(n-1))/(n/(n-1)+n/2*(1+alfa*x**(1/n))**2)-x**(1-1/n)
            return(y)
        rc = opt.fsolve(y, 0.5)
        r=p2_pa/p1_pa
        if rc >= r: pr=rc
        else: pr=r
        Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
        Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
        wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
        return wi
    def w2(p2_atm):
        return wi(p2_atm)-i
    def p2_atm(w2):
        p2_atm=opt.fsolve(w2,p1_atm/1.00001)
        return p2_atm
    p1_atm_a= np.arange(10,20,0.1)
    q=np.array([])
    for p1_atm in p1_atm_a:
        p2=p2_atm(w2)
        q=np.append(q,p2)
    plt.plot(p1_atm_a,q,label='w=%1.1f'%i+ 'кг/час')
plt.title('Давление на выходе в зависимости от расхода и давления на входе')
plt.xlabel('Давление на входе, атм')
plt.ylabel('Давление на выходе, атм')   
plt.legend()
plt.show() 

""" 
расчет перепада давления в зависимости от расхода

"""
def wi(f):
    p1_atm=12
    k=1.3 # показатель адиабаты
    Cd=0.75 # дисчарч коэф для штуцера
    g_c=9.8 # ускорение свободного падения
    p1_pa=p1_atm*10**5 #
    Vg1=57.8 # удельный объем пара
    Vl=0.001 # удельный объем жидкости (в данном случае воды)
    CL=4.176 # удельная теплоемкость жидкости кДж/кг/К
    Cvg=1.539 # удельная теплоемкость пара кДж/кг/К
    d2=0.005 # диаметр штуцера
    xg=1 #  массовая доля пара
    R=(1+xg*(Vg1/Vl-1))**0.5*(1+0.6/2.718**(5*xg))
    alfa=R*(1-xg)*Vl/(xg*Vg1)
    A2=3.14*d2**2/4
    n=(xg*k*Cvg+(1-xg)*CL)/(xg*Cvg+(1-xg)*CL)
    def y(x):
        y=(alfa*(1-x)+n/(n-1))/(n/(n-1)+n/2*(1+alfa*x**(1/n))**2)-x**(1-1/n)
        return(y)
    rc = opt.fsolve(y, 0.5)
    r=1-f/p1_pa
    if rc >= r: pr=rc
    else: pr=r
    Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
    Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
    wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
    return wi
i_a=np.arange(0,150,5)
for i in i_a:
    def w2(f):
        return wi(f)-i
    def f(w2):
        f=opt.fsolve(w2,0)
        return f
    q=np.array([])
    for i in i_a:
        f1=f(w2)
        q=np.append(q,f1)
plt.plot(q/100000, i_a)
plt.title('Перепад давления в зависимости от расхода')
plt.ylabel('Расход, кг/час')
plt.xlabel('Перепад давления, атм')   
plt.show() 