# -*- coding: utf-8 -*-
"""
Created on Mon Aug 20 15:43:09 2018

@author: Артём
"""

import scipy.optimize as opt
from pyXSteam.XSteam import XSteam
steamTable = XSteam(XSteam.UNIT_SYSTEM_MKS)

def w_saturatedsteam_choke_kghr(p1_atm, p2_atm, X, d0_mm=5, d1_mm=100):
    
    """
    расчет расхода насыщенного пара через штуцер по методике Чиена (Миллера)
    p1_atm - давление на входе в штуцер, атм
    p2_atm - давление на выходе из штуцера, атм
    d0_mm - диаметр штуцера, мм
    d1_mm -  диаметр трубы, мм
    X - cухость пара, д.ед.
    """
    C0 =0.8 # коэффициент разряда штуцера
    Y1=0.89487 # коэффициент расширения пара,считают его постоянным,зависит от давлений и диаметров
    A=0.99998 # эмпирические коэффициенты, рассчитаны из графиков по статье
    B=1.38
    vfg_m3kg=steamTable.vV_p(p1_atm) # удельный объем пара при заданном давлении м3/кг
    vf_m3kg=steamTable.vL_p(p1_atm) # удельный объем жидкости при заданном давлении м3/кг
    vexp_m3kg=A*vfg_m3kg*X**B+vf_m3kg #удельный объем смеси м3/кг
    ro_kgm3=1/vexp_m3kg #плотность смеси кг/м3
    beta=d0_mm/d1_mm
    f_atm=p1_atm-p2_atm 
    f_kPa=f_atm*100
    w_kgsec=3.512407*10**(-5)*C0*Y1*d0_mm**2*(f_kPa*ro_kgm3)**0.5/(1-beta**4)**0.5
    w_kghr=w_kgsec*3600
    return w_kghr

def p_saturatedsteam_upchoke_atm(w_kghr,p2_atm,X,d0_mm=5, d1_mm=100):   
    """
    расчет давления насыщенного пара перед клапаном по методике Чиена (Миллера)
    w_kghr - расход насыщенного пара, кг/час
    p2_atm - давление на выходе из штуцера, атм
    X - cухость пара, д.ед.
    d0_mm - диаметр штуцера, мм
    d1_mm -  диаметр трубы, мм
    """
    def w2(p1_atm):
        return w_saturatedsteam_choke_kghr(p1_atm,p2_atm,X,d0_mm=5, d1_mm=100)-w_kghr
    return opt.fsolve(w2,p2_atm)

def p_saturatedsteam_downchoke_atm( w_kghr, p1_atm, X,d0_mm=5,d1_mm=100):
    """
    расчет давления на выходе от давления на входе при разных расходах по методике Чиена (Миллера)
    p1_atm - давление на входе в штуцер, атм
    w_kghr - расход пара, кг/час
    X -  сухость пара, д.ед.
    d0_mm - диаметр штуцера, мм
    d1_mm - диаметр трубы, мм
    """
    C0 =0.8 # коэффициент разряда штуцера
    Y1=0.89487 # коэффициент расширения пара
    A=0.99998 # эмпирические коэффициенты, рассчитаны из графиков по статье
    B=1.38
    vfg_m3kg=steamTable.vV_p(p1_atm) # удельный объем пара при заданном давлении м3/кг
    vf_m3kg=steamTable.vL_p(p1_atm) # удельный объем жидкости при заданном давлении м3/кг
    vexp_m3kg=A*vfg_m3kg*X**B+vf_m3kg #удельный объем смеси м3/кг
    ro_kgm3=1/vexp_m3kg #плотность смеси кг/м3
    beta=d0_mm/d1_mm
    w_kgsec=w_kghr/3600
    f_kPa=w_kgsec**2*(1-beta**4)*10**10/(12.337*C0**2*Y1**2*d0_mm**4*ro_kgm3)
    p2_atm=p1_atm-f_kPa/100
    return p2_atm

def f_saturatedsteam_difpressurechoke_atm(w_kghr,p1_atm,X,d0_mm=5, d1_mm=100):
    """
    расчет разницы давлений до и после штуцера насыщенного пара по методике Чиена (Миллера)
    w_kghr - расход насыщенного пара, кг/час
    p1_atm - давление на входе в штуцер, атм
    X - cухость пара, д.ед.
    d0_mm - диаметр штуцера, мм
    d1_mm -  диаметр трубы, мм
    """
    def w2(f):
        return w_saturatedsteam_choke_kghr(p1_atm,p1_atm-f,X,d0_mm=5, d1_mm=100)-w_kghr
    return opt.fsolve(w2,0)

def q_saturatsteam_choke_kghr(p1_atm, p2_atm, xg, dchoke_mm=5):
    """
    расчет расхода насыщенного пара через штуцер по методике Альсафран 
    p1_atm - давление на входе в штуцер, атм
    p2_atm - давление на выходе из штуцера, атм
    dchoke_mm - диаметр штуцера,мм
    xg - массовая доля пара, д.ед.
    """
    if xg <= 0: xg=0.0001
    Cd=0.75 # дисчарч коэффициент для штуцера (коэффициент разряда штуцера)
    g_c=9.8 # ускорение свободного падения
    p1_pa=p1_atm*100 # давление до штуцера, Па
    p2_pa=p2_atm*100 # давление после штуцера, Па
    Vg1=steamTable.vV_p(p1_atm)  # удельный объем пара, м3/кг
    Vl=steamTable.vL_p(p1_atm)  # удельный объем воды, м3/кг
    CL=steamTable.CvL_p(p1_atm)  # удельная теплоемкость воды, кДж/кг/К
    Cvg=steamTable.CvV_p(p1_atm)  # удельная теплоемкость пара при постоянном объеме, кДж/кг/К 
    Cpg=steamTable.CpV_p(p1_atm)  # удельная теплоемкость пара при постоянном давлении, кДж/кг/К 
    k=Cpg/Cvg # показатель адиабаты
    d2=dchoke_mm/1000 #диаметр клапана, м
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
    if pr>1: pr=1
    Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
    Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
    wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
    return wi

def q_critsaturatsteam_choke_kghr(p1_atm, xg, dchoke_mm=5):
    """
    расчет критического расхода насыщенного пара через штуцер по методике Альсафран 
    p1_atm - давление на входе в штуцер, атм
    dchoke_mm - диаметр штуцера,мм
    xg - массовая доля пара, д.ед.
    """
    if xg <= 0: xg=0.0001
    Cd=0.75 # дисчарч коэффициент для штуцера (коэффициент разряда штуцера)
    g_c=9.8 # ускорение свободного падения
    p1_pa=p1_atm*100 # давление до штуцера, Па
    Vg1=steamTable.vV_p(p1_atm)  # удельный объем пара, м3/кг
    Vl=steamTable.vL_p(p1_atm)  # удельный объем воды, м3/кг
    CL=steamTable.CvL_p(p1_atm)  # удельная теплоемкость воды, кДж/кг/К
    Cvg=steamTable.CvV_p(p1_atm)  # удельная теплоемкость пара при постоянном объеме, кДж/кг/К 
    Cpg=steamTable.CpV_p(p1_atm)  # удельная теплоемкость пара при постоянном давлении, кДж/кг/К 
    k=Cpg/Cvg # показатель адиабаты
    d2=dchoke_mm/1000 #диаметр клапана, м
    R=(1+xg*(Vg1/Vl-1))**0.5*(1+0.6/2.718**(5*xg))
    alfa=R*(1-xg)*Vl/(xg*Vg1)
    A2=3.14*d2**2/4
    n=(xg*k*Cvg+(1-xg)*CL)/(xg*Cvg+(1-xg)*CL)
    def y(x):
        y=(alfa*(1-x)+n/(n-1))/(n/(n-1)+n/2*(1+alfa*x**(1/n))**2)-x**(1-1/n)
        return(y)
    rc = opt.fsolve(y, 0.5)
    pr=rc
    if pr>1: pr=1
    Ab=n/(n-1)*(1-pr**((n-1)/n))+alfa*(1-pr)
    Ac=xg*Vg1*(pr**(-1/n)+alfa)**2*(xg+1/R*(1-xg))
    wi=A2*(288*g_c*Cd**2*p1_pa*Ab/Ac)**0.5*3600
    return wi

def p_saturatsteam_upchoke_atm(w_kghr,p2_atm,xg,dchoke_mm=5):   
    """
    расчет давления насыщенного пара перед клапаном по методике Альсафран
    w_kghr - расход насыщенного пара, кг/час
    p2_atm - давление на выходе из штуцера, атм
    xg - cухость пара, д.ед.
    dchoke_mm - диаметр штуцера, мм
    """
    def w2(p1_atm):
        return q_saturatsteam_choke_kghr(p1_atm, p2_atm, xg, dchoke_mm=5)-w_kghr
    return opt.fsolve(w2,p2_atm)

def p_saturatsteam_downchoke_atm(w_kghr,p1_atm,xg,dchoke_mm=5):   
    """
    расчет давления насыщенного пара перед клапаном по методике Альсафран
    w_kghr - расход насыщенного пара, кг/час
    p1_atm - давление на входе в штуцер, атм
    xg - cухость пара, д.ед.
    dchoke_mm - диаметр штуцера, мм
    """
    def w2(p2_atm):
        return q_saturatsteam_choke_kghr(p1_atm, p2_atm, xg, dchoke_mm=5)-w_kghr
    p2=opt.fsolve(w2,p1_atm-0.00001)
    if w_kghr>q_critsaturatsteam_choke_kghr(p1_atm, xg): 
        print('Ошибка! Слишком большой расход для такого входного давления!')
        p2=0
    return p2

def f_saturatsteam_difpressurechoke_atm(w_kghr,p1_atm,xg,dchoke_mm=5):
    """
    расчет разницы давлений до и после штуцера насыщенного пара по методике Aльсафран
    w_kghr - расход насыщенного пара, кг/час
    p1_atm - давление на входе в штуцер, атм
    xg - cухость пара, д.ед.
    dchoke_mm - диаметр штуцера, мм
    """
    def w2(f):
        return q_saturatsteam_choke_kghr(p1_atm, p1_atm-f, xg, dchoke_mm=5)-w_kghr
    f=opt.fsolve(w2,0)
    if w_kghr>q_critsaturatsteam_choke_kghr(p1_atm, xg): 
        print('Ошибка! Слишком большой расход для такого входного давления!')
        f=0
    return f

def W_ssteam_choke_kghr(p1_atm,p2_atm, fg, T_C=20, d2_mm=5):
   
    """
    расчет расхода насыщенного пара через штуцер по методике Перкинса
    p1_atm - давление на входе в штуцер, атм
    p2_atm - давление на выходе из штуцера, атм
    T_C - температура, С
    d2_mm - диаметр штуцера, мм
    fg - весовая доля газа в потоке или по другому сухость пара, д.ед.
    """
    if fg<=0: fg=0.00001
    fw=1-fg # весовая доля воды в потоке
    M_m=18 # молекулярный вес пара, моль пара
    z=0.999 # коэффициент сжимаемости пара
    rog_kgm3=steamTable.rhoV_p(p1_atm) # плотность пара кг/м3
    row_kgm3=steamTable.rhoL_p(p1_atm) # плотность воды   кг/м3
    Cvw=0.24*778.169*steamTable.CvL_p(p1_atm)  # удельная теплоемкость воды 1 кДж/кгК= 1*0,24*778.169=(ft-Ibf)/(lbm- OF)
    Cvg=0.24*778.169*steamTable.CvV_p(p1_atm)  # удельная теплоемкость пара при постоянном объеме, кДж/кг/К 
    Cpg=0.24*778.169*steamTable.CpV_p(p1_atm)  # удельная теплоемкость пара при постоянном давлении, кДж/кг/К
    d1_mm=100 #диаметр трубы до штуцера мм
    g_c=32.17 #(lbm-ft)/(lbf-second^2)
    row=row_kgm3*0.062428 # плотность воды  Ibm/ft3
    #T_F=T_C*1.8 + 32 # температура, F
    R_ftIbtlbmmolR=1545.348 # универсальная газовая постоянная  (ft-Ibf)/(lbm mol-R)
    F=Cpg/Cvg # показатель адиабаты
    p1_psia=p1_atm*14.2233 # давление перед штуцером в psia
    p2_psia=p2_atm*14.2233 # давление за штуцером в psia
    d1_ft=d1_mm/304.8 # диаметр трубы в ft
    d2_ft=d2_mm/304.8 # диаметр штуцера в ft
    v1_ft3lbm=16.01845/rog_kgm3 # удельный объем газа ft3/Ibm
    alf=(1/v1_ft3lbm)*fw/row
    A1_ft2=3.14*d1_ft**2/4
    A2_ft2=3.14*d2_ft**2/4
    n=(fg*F*Cvg+fw*Cvw)/(fg*Cvg+fw*Cvw)
    lambd=fg+((fg*Cvg+fw*Cvw)*M_m/(z*R_ftIbtlbmmolR))
    i=0
    pr1=0.1
    pr11=0.2
    """
    Пересчитываю отношение давлений столько раз, пока разница между следующим и предыдущим не будет очень маленькой.
    Отношение давлений служит для того, чтобы найти давление в штуцере.
    """
    while abs(pr1-pr11)>0.001 and i<10: # пересчитываем удельные теплоемкости и давление в штуцере, пока они не будут соответствовать A*(B+C)=D*E
        def qw(pr2):
            A=2*lambd*(1-pr2**((n-1)/n))+2*alf*(1-pr2)
            B=(1-((A2_ft2/A1_ft2)**2)*((fg+alf)/((fg*pr2**(-1/n))+alf))**2)*(fg/n*pr2**(-(1+n)/n))
            C=(A2_ft2/A1_ft2)**2*fg/n*(fg+alf)**2*pr2**(-(1+n)/n)/((fg*pr2**(-1/n))+alf)**2
            D=(1-(A2_ft2/A1_ft2)*((fg+alf)/((fg*pr2**(-1/n))+alf))**2)*((fg*pr2**(-1/n))+alf)
            E=lambd*(n-1)/n*pr2**(-1/n)+alf
            return A*(B+C)-D*E
        pr1=opt.fsolve(qw,0.01) #находим отношение давления в штуцере и давления перед штуцером
        p3_psia=p1_psia*pr1 #находим давление в штуцере
        pmid_atm=(p1_psia+p3_psia)/2/14.2233 # среднее давление в штуцере и перед штуцером, пересчитываем удельные теплоемкости для этого давления
        Cvw=0.24*778.169*steamTable.CvL_p(pmid_atm)  # удельная теплоемкость жидкости 1 кДж/кгК= 1*0,24*778.169=(ft-Ibf)/(lbm- OF)
        Cvg=0.24*778.169*steamTable.CvV_p(pmid_atm)  # удельная теплоемкость пара при постоянном объеме, кДж/кг/К 
        Cpg=0.24*778.169*steamTable.CpV_p(pmid_atm)  # удельная теплоемкость пара при постоянном давлении, кДж/кг/К
        F=Cpg/Cvg
        n=(fg*F*Cvg+fw*Cvw)/(fg*Cvg+fw*Cvw)
        lambd=fg+((fg*Cvg+fw*Cvw)*M_m/(z*R_ftIbtlbmmolR))
        pr11=opt.fsolve(qw,0.01)
        i=i+1
    if p3_psia>p2_psia:pr= p3_psia/p1_psia
    else: pr=p2_psia/p1_psia
    if pr>1: pr=1
    Ab=(lambd*(1-pr**((n-1)/n))+alf*(1-pr))/((1-((A2_ft2/A1_ft2)**2)*((fg+alf)/((fg*pr**(-1/n))+alf))**2)*((fg*pr**(-1/n))+alf)**2)
    wi_lbmsec=A2_ft2*((288*g_c*p1_psia/v1_ft3lbm)*Ab)**0.5
    wi_kghr=wi_lbmsec*0.45359*3600 # перевод в кг/час
    #T2_F=(T_F+460)*pr1**((n-1)/n)-460 #Температура в штуцере в F
    #T2_C=(T2_F-32)/1.8 #Температура в штуцере в С
    return wi_kghr

def W_critssteam_choke_kghr(p1_atm, fg, T_C=20, d2_mm=5):
   
    """
    расчет критического расхода насыщенного пара через штуцер по методике Перкинса
    p1_atm - давление на входе в штуцер, атм
    p2_atm - давление на выходе из штуцера, атм
    T_C - температура, С
    d2_mm - диаметр штуцера, мм
    fg - весовая доля газа в потоке или по другому сухость пара, д.ед.
    """
    if fg<=0: fg=0.00001
    fw=1-fg # весовая доля воды в потоке
    M_m=18 # молекулярный вес пара, моль пара
    z=0.999 # коэффициент сжимаемости пара
    rog_kgm3=steamTable.rhoV_p(p1_atm) # плотность пара кг/м3
    row_kgm3=steamTable.rhoL_p(p1_atm) # плотность воды   кг/м3
    Cvw=0.24*778.169*steamTable.CvL_p(p1_atm)  # удельная теплоемкость воды 1 кДж/кгК= 1*0,24*778.169=(ft-Ibf)/(lbm- OF)
    Cvg=0.24*778.169*steamTable.CvV_p(p1_atm)  # удельная теплоемкость пара при постоянном объеме, кДж/кг/К 
    Cpg=0.24*778.169*steamTable.CpV_p(p1_atm)  # удельная теплоемкость пара при постоянном давлении, кДж/кг/К
    d1_mm=100 #диаметр трубы до штуцера мм
    g_c=32.17 #(lbm-ft)/(lbf-second^2)
    row=row_kgm3*0.062428 # плотность воды  Ibm/ft3
    R_ftIbtlbmmolR=1545.348 # универсальная газовая постоянная  (ft-Ibf)/(lbm mol-R)
    F=Cpg/Cvg # показатель адиабаты
    p1_psia=p1_atm*14.2233 # давление перед штуцером в psia
    d1_ft=d1_mm/304.8 # диаметр трубы в ft
    d2_ft=d2_mm/304.8 # диаметр штуцера в ft
    v1_ft3lbm=16.01845/rog_kgm3 # удельный объем газа ft3/Ibm
    alf=(1/v1_ft3lbm)*fw/row
    A1_ft2=3.14*d1_ft**2/4
    A2_ft2=3.14*d2_ft**2/4
    n=(fg*F*Cvg+fw*Cvw)/(fg*Cvg+fw*Cvw)
    lambd=fg+((fg*Cvg+fw*Cvw)*M_m/(z*R_ftIbtlbmmolR))
    i=0
    pr1=0.1
    pr11=0.2
    """
    Пересчитываю отношение давлений столько раз, пока разница между следующим и предыдущим не будет очень маленькой.
    Отношение давлений служит для того, чтобы найти давление в штуцере.
    """
    while abs(pr1-pr11)>0.001 and i<10: # пересчитываем удельные теплоемкости и давление в штуцере, пока они не будут соответствовать A*(B+C)=D*E
        def qw(pr2):
            A=2*lambd*(1-pr2**((n-1)/n))+2*alf*(1-pr2)
            B=(1-((A2_ft2/A1_ft2)**2)*((fg+alf)/((fg*pr2**(-1/n))+alf))**2)*(fg/n*pr2**(-(1+n)/n))
            C=(A2_ft2/A1_ft2)**2*fg/n*(fg+alf)**2*pr2**(-(1+n)/n)/((fg*pr2**(-1/n))+alf)**2
            D=(1-(A2_ft2/A1_ft2)*((fg+alf)/((fg*pr2**(-1/n))+alf))**2)*((fg*pr2**(-1/n))+alf)
            E=lambd*(n-1)/n*pr2**(-1/n)+alf
            return A*(B+C)-D*E
        pr1=opt.fsolve(qw,0.01) #находим отношение давления в штуцере и давления перед штуцером
        p3_psia=p1_psia*pr1 #находим давление в штуцере
        pmid_atm=(p1_psia+p3_psia)/2/14.2233 # среднее давление в штуцере и перед штуцером, пересчитываем удельные теплоемкости для этого давления
        Cvw=0.24*778.169*steamTable.CvL_p(pmid_atm)  # удельная теплоемкость жидкости 1 кДж/кгК= 1*0,24*778.169=(ft-Ibf)/(lbm- OF)
        Cvg=0.24*778.169*steamTable.CvV_p(pmid_atm)  # удельная теплоемкость пара при постоянном объеме, кДж/кг/К 
        Cpg=0.24*778.169*steamTable.CpV_p(pmid_atm)  # удельная теплоемкость пара при постоянном давлении, кДж/кг/К
        F=Cpg/Cvg
        n=(fg*F*Cvg+fw*Cvw)/(fg*Cvg+fw*Cvw)
        lambd=fg+((fg*Cvg+fw*Cvw)*M_m/(z*R_ftIbtlbmmolR))
        pr11=opt.fsolve(qw,0.01)
        i=i+1
    pr= p3_psia/p1_psia
    if pr>1: pr=1
    Ab=(lambd*(1-pr**((n-1)/n))+alf*(1-pr))/((1-((A2_ft2/A1_ft2)**2)*((fg+alf)/((fg*pr**(-1/n))+alf))**2)*((fg*pr**(-1/n))+alf)**2)
    wi_lbmsec=A2_ft2*((288*g_c*p1_psia/v1_ft3lbm)*Ab)**0.5
    wi_kghr=wi_lbmsec*0.45359*3600 # перевод в кг/час
    return wi_kghr

def p_ssteam_upchoke_atm(w_kghr,p2_atm,fg,d2_mm=5):   
    """
    расчет давления насыщенного пара перед клапаном по методике Перкинса
    w_kghr - расход насыщенного пара, кг/час
    p2_atm - давление на выходе из штуцера, атм
    fg - cухость пара, д.ед.
    d2_mm - диаметр штуцера, мм
    """
    def w2(p1_atm):
        return W_ssteam_choke_kghr(p1_atm, p2_atm, fg, d2_mm=5)-w_kghr
    return opt.fsolve(w2,p2_atm)

def p_ssteam_downchoke_atm(w_kghr,p1_atm,fg,d2_mm=5):   
    """
    расчет давления насыщенного пара перед клапаном по методике Перкинса
    w_kghr - расход насыщенного пара, кг/час
    p1_atm - давление на входе в штуцер, атм
    fg - cухость пара, д.ед.
    d2_mm - диаметр штуцера, мм
    """
    def w2(p2_atm):
        return W_ssteam_choke_kghr(p1_atm, p2_atm, fg, d2_mm=5)-w_kghr
    p2=opt.fsolve(w2,p1_atm-0.00001)
    if w_kghr>W_critssteam_choke_kghr(p1_atm, fg): 
        print('Ошибка! Слишком большой расход для такого входного давления!')
        p2=0
    return p2

def f_ssteam_difpressurechoke_atm(w_kghr,p1_atm,fg,d2_mm=5):
    """
    расчет разницы давлений до и после штуцера насыщенного пара по методике Перкинса
    w_kghr - расход насыщенного пара, кг/час
    p1_atm - давление на входе в штуцер, атм
    fg - cухость пара, д.ед.
    d2_mm - диаметр штуцера, мм
    """
    def w2(f):
        return W_ssteam_choke_kghr(p1_atm, p1_atm-f, fg)-w_kghr
    f=opt.fsolve(w2,0)
    if w_kghr>W_critssteam_choke_kghr(p1_atm, fg): 
        print('Ошибка! Слишком большой расход для такого входного давления!')
        f=0
    return f